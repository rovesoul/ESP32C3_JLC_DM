#include "wifi_provisioning.h"
#include "OLED.h"
#include "string.h"
#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "lwip/inet.h"
#include "lwip/ip_addr.h"

// NVS 命名空间和键名
#define NVS_NAMESPACE "wifi_config"
#define NVS_KEY_SSID "wifi_ssid"
#define NVS_KEY_PASSWORD "wifi_password"

// SoftAP 配置
#define PROVISIONING_SSID_PREFIX "ESP32-Setup"
#define PROVISIONING_PASSWORD "12345678"
#define PROVISIONING_CHANNEL 1
#define PROVISIONING_MAX_CONN 4

// 超时配置（5分钟超时）
#define PROVISIONING_TIMEOUT_MS (5 * 60 * 1000)

static const char *TAG = "wifi_prov";

// 事件组标志位
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_PROV_DONE_BIT BIT1

static EventGroupHandle_t s_wifi_event_group;
static httpd_handle_t s_provisioning_server = NULL;

// 全局标志: 是否处于配网模式
static bool s_is_provisioning_mode = false;

/**
 * @brief 生成配网热点名称（添加MAC后缀以区分设备）
 */
static void generate_provisioning_ssid(char *ssid, size_t max_len) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(ssid, max_len, "%s_%02X%02X%02X",
             PROVISIONING_SSID_PREFIX, mac[3], mac[4], mac[5]);
}

/**
 * @brief 显示配网模式OLED界面(4行内容)
 * @param ap_ssid 配网热点名称
 * @param ap_password 配网热点密码
 * @param ip_address 配网IP地址
 */
static void display_provisioning_interface(const char *ap_ssid, const char *ap_password, const char *ip_address) {
    // 清屏
    OLED_Clear();

    // 第1行: "SET YOUR WIFI" (居中显示)
    char line1[] = "SET YOUR WIFI";
    int16_t x1 = 64 - (strlen(line1) * 6) / 2;  // 居中计算 (字体宽度6像素)
    OLED_ShowString(x1 > 0 ? x1 : 0, 0, line1, OLED_6X8);

    // 第2行: WiFi SSID (左对齐)
    char line2[32];
    snprintf(line2, sizeof(line2), "SSID:%s", ap_ssid);
    OLED_ShowString(0, 16, line2, OLED_6X8);

    // 第3行: WiFi Password (左对齐)
    char line3[32];
    snprintf(line3, sizeof(line3), "PASS:%s", ap_password);
    OLED_ShowString(0, 32, line3, OLED_6X8);

    // 第4行: IP Address (左对齐)
    char line4[32];
    snprintf(line4, sizeof(line4), "IP:%s", ip_address);
    OLED_ShowString(0, 48, line4, OLED_6X8);

    // 更新显示
    OLED_Update();

    ESP_LOGI(TAG, "OLED配网界面已显示");
}

/**
 * @brief 检查NVS中是否已保存WiFi凭据
 */
bool wifi_provisioning_has_config(void) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t required_size;
        err = nvs_get_str(handle, NVS_KEY_SSID, NULL, &required_size);
        nvs_close(handle);
        return (err == ESP_OK);
    }
    return false;
}

/**
 * @brief 检查是否处于配网模式
 * @return true 如果当前处于配网模式
 */
bool wifi_provisioning_is_active(void) {
    return s_is_provisioning_mode;
}

/**
 * @brief 从NVS加载WiFi凭据
 */
esp_err_t wifi_provisioning_load_config(char *ssid, char *password,
                                        size_t ssid_buf_size, size_t password_buf_size) {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        return ESP_ERR_NOT_FOUND;
    }

    size_t required_size = ssid_buf_size;
    err = nvs_get_str(handle, NVS_KEY_SSID, ssid, &required_size);
    if (err != ESP_OK) {
        nvs_close(handle);
        return err;
    }

    required_size = password_buf_size;
    err = nvs_get_str(handle, NVS_KEY_PASSWORD, password, &required_size);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Loaded WiFi config from NVS: SSID=%s", ssid);
    }

    return err;
}

/**
 * @brief 保存WiFi凭据到NVS
 */
esp_err_t wifi_provisioning_save_config(const char *ssid, const char *password) {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_str(handle, NVS_KEY_SSID, ssid);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save SSID: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_set_str(handle, NVS_KEY_PASSWORD, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save password: %s", esp_err_to_name(err));
        nvs_close(handle);
        return err;
    }

    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved WiFi config to NVS: SSID=%s", ssid);
    }

    return err;
}

/**
 * @brief 清除已保存的WiFi凭据
 */
esp_err_t wifi_provisioning_clear_config(void) {
    nvs_handle_t handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    nvs_erase_all(handle);
    err = nvs_commit(handle);
    nvs_close(handle);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Cleared WiFi config from NVS");
    }

    return err;
}

// HTTP GET处理：返回配网页面
static esp_err_t provisioning_get_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "配网页面被访问");
    const char *html_page =
        "<!DOCTYPE html>"
        "<html>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>WiFi配网 - ESP32</title>"
        "<style>"
        "body { font-family: Arial, sans-serif; margin: 40px; background: #f5f5f5; }"
        ".container { max-width: 400px; margin: 0 auto; background: white; padding: 30px; border-radius: 10px; box-shadow: 0 2px 10px rgba(0,0,0,0.1); }"
        "h1 { color: #333; text-align: center; margin-bottom: 30px; }"
        ".form-group { margin-bottom: 20px; }"
        "label { display: block; margin-bottom: 8px; color: #555; font-weight: bold; }"
        "input[type='text'], input[type='password'] { width: 100%; padding: 12px; border: 1px solid #ddd; border-radius: 5px; box-sizing: border-box; font-size: 14px; }"
        "button { width: 100%; padding: 14px; background: #4CAF50; color: white; border: none; border-radius: 5px; font-size: 16px; font-weight: bold; cursor: pointer; margin-top: 10px; }"
        "button:hover { background: #45a049; }"
        ".status { margin-top: 20px; padding: 15px; border-radius: 5px; text-align: center; display: none; }"
        ".status.success { background: #d4edda; color: #155724; }"
        ".status.error { background: #f8d7da; color: #721c24; }"
        ".status.info { background: #d1ecf1; color: #0c5460; }"
        "</style>"
        "</head>"
        "<body>"
        "<div class='container'>"
        "<h1>📡 WiFi配网</h1>"
        "<form id='wifiForm'>"
        "<div class='form-group'>"
        "<label for='ssid'>WiFi名称 (SSID)</label>"
        "<input type='text' id='ssid' name='ssid' required placeholder='请输入WiFi名称'>"
        "</div>"
        "<div class='form-group'>"
        "<label for='password'>WiFi密码</label>"
        "<input type='password' id='password' name='password' required placeholder='请输入WiFi密码'>"
        "</div>"
        "<button type='submit' id='submitBtn'>连接WiFi</button>"
        "</form>"
        "<div id='status' class='status'></div>"
        "</div>"
        "<script>"
        "document.getElementById('wifiForm').addEventListener('submit', async function(e) {"
        "  e.preventDefault();"
        "  const ssid = document.getElementById('ssid').value;"
        "  const password = document.getElementById('password').value;"
        "  const statusDiv = document.getElementById('status');"
        "  const submitBtn = document.getElementById('submitBtn');"
        "  "
        "  statusDiv.style.display = 'block';"
        "  statusDiv.className = 'status info';"
        "  statusDiv.textContent = '正在连接WiFi，请稍候...';"
        "  submitBtn.disabled = true;"
        "  "
        "  try {"
        "    const response = await fetch('/connect', {"
        "      method: 'POST',"
        "      headers: { 'Content-Type': 'application/json' },"
        "      body: JSON.stringify({ ssid: ssid, password: password })"
        "    });"
        "    "
        "    const result = await response.json();"
        "    "
        "    if (result.status === 'success') {"
        "      statusDiv.className = 'status success';"
        "      statusDiv.textContent = '✅ WiFi连接成功！设备即将重启，请稍候...';"
        "      setTimeout(() => { location.reload(); }, 3000);"
        "    } else {"
        "      statusDiv.className = 'status error';"
        "      statusDiv.textContent = '❌ 连接失败: ' + (result.message || '未知错误');"
        "      submitBtn.disabled = false;"
        "    }"
        "  } catch (error) {"
        "    statusDiv.className = 'status error';"
        "    statusDiv.textContent = '❌ 请求失败: ' + error.message;"
        "    submitBtn.disabled = false;"
        "  }"
        "});"
        "</script>"
        "</body>"
        "</html>";

    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, html_page, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP POST处理：接收WiFi配置并尝试连接
static esp_err_t provisioning_connect_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "收到配网请求");
    char buf[256];
    int total_len = req->content_len;
    int cur_len = 0;

    ESP_LOGI(TAG, "Content length: %d", total_len);

    if (total_len >= sizeof(buf)) {
        ESP_LOGE(TAG, "请求太大");
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too large");
        return ESP_FAIL;
    }

    // 读取请求体
    while (cur_len < total_len) {
        int received = httpd_req_recv(req, buf + cur_len, sizeof(buf) - cur_len);
        ESP_LOGI(TAG, "收到 %d 字节, 总计 %d/%d", received, cur_len, total_len);
        if (received <= 0) {
            ESP_LOGE(TAG, "读取请求失败");
            return ESP_FAIL;
        }
        cur_len += received;
    }
    buf[cur_len] = '\0';

    ESP_LOGI(TAG, "收到数据: %s", buf);

    // 解析JSON
    char ssid[64] = {0};
    char password[64] = {0};
    int parsed = sscanf(buf, "{\"ssid\":\"%63[^\"]\",\"password\":\"%63[^\"]\"}", ssid, password);
    ESP_LOGI(TAG, "JSON解析结果: %d, SSID=%s, Password长度=%d", parsed, ssid, (int)strlen(password));

    if (parsed != 2) {
        ESP_LOGE(TAG, "JSON格式错误");
        const char *response = "{\"status\":\"error\",\"message\":\"Invalid JSON format\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "收到WiFi配置: SSID=%s", ssid);

    // 保存到NVS
    ESP_LOGI(TAG, "正在保存到NVS...");
    esp_err_t err = wifi_provisioning_save_config(ssid, password);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "保存到NVS失败: %s", esp_err_to_name(err));
        const char *response = "{\"status\":\"error\",\"message\":\"Failed to save config\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "已保存到NVS");

    // 尝试连接WiFi
    ESP_LOGI(TAG, "正在连接WiFi: %s", ssid);
    wifi_config_t wifi_config = {0};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_connect();

    ESP_LOGI(TAG, "等待WiFi连接结果（最多10秒）...");
    // 等待连接结果（最多10秒）
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_CONNECTED_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "WiFi连接成功！");
        const char *response = "{\"status\":\"success\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);

        ESP_LOGI(TAG, "Provisioning successful!");
        xEventGroupSetBits(s_wifi_event_group, WIFI_PROV_DONE_BIT);
        return ESP_OK;
    } else {
        // 连接失败，清除保存的配置
        ESP_LOGE(TAG, "WiFi连接失败，清除配置");
        wifi_provisioning_clear_config();

        const char *response = "{\"status\":\"error\",\"message\":\"WiFi connection failed\"}";
        httpd_resp_set_type(req, "application/json");
        httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
        return ESP_FAIL;
    }
}

// 配网模式的事件处理
static void provisioning_event_handler(void* arg, esp_event_base_t event_base,
                                     int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "Connected to AP during provisioning");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ESP_LOGI(TAG, "Got IP during provisioning");
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Disconnected from AP during provisioning");
    }
}

// 启动配网HTTP服务器
static void start_provisioning_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    esp_err_t err = httpd_start(&s_provisioning_server, &config);
    if (err == ESP_OK) {
        httpd_uri_t uri_get = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = provisioning_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(s_provisioning_server, &uri_get);

        httpd_uri_t uri_connect = {
            .uri = "/connect",
            .method = HTTP_POST,
            .handler = provisioning_connect_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(s_provisioning_server, &uri_connect);

        ESP_LOGI(TAG, "✅ Provisioning HTTP server started on port 80");
    } else {
        ESP_LOGE(TAG, "❌ Failed to start HTTP server: %s", esp_err_to_name(err));
    }
}

/**
 * @brief 启动WiFi配网模式
 */
esp_err_t wifi_provisioning_start(void) {
    s_wifi_event_group = xEventGroupCreate();
    s_is_provisioning_mode = true;  // 设置配网模式标志

    // 创建AP接口的netif（重要！）
    esp_netif_create_default_wifi_ap();

    // 初始化WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &provisioning_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &provisioning_event_handler, NULL));

    // 生成配网热点名称
    char ap_ssid[32];
    generate_provisioning_ssid(ap_ssid, sizeof(ap_ssid));
    ESP_LOGI(TAG, "Starting provisioning AP: %s (Password: %s)", ap_ssid, PROVISIONING_PASSWORD);

    // 配置SoftAP
    wifi_config_t ap_config = {
        .ap = {
            .ssid_len = strlen(ap_ssid),
            .channel = PROVISIONING_CHANNEL,
            .max_connection = PROVISIONING_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strncpy((char *)ap_config.ap.ssid, ap_ssid, sizeof(ap_config.ap.ssid));
    strncpy((char *)ap_config.ap.password, PROVISIONING_PASSWORD, sizeof(ap_config.ap.password));

    ESP_LOGI(TAG, "正在设置WiFi模式为AP+STA...");
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    ESP_LOGI(TAG, "正在配置AP参数...");
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));

    ESP_LOGI(TAG, "正在启动WiFi...");
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "✅ WiFi AP已启动");

    // 启动HTTP服务器
    start_provisioning_server();

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "WiFi Provisioning Mode Activated");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "1. 连接WiFi: %s", ap_ssid);
    ESP_LOGI(TAG, "2. 密码: %s", PROVISIONING_PASSWORD);
    ESP_LOGI(TAG, "3. 打开浏览器访问: http://192.168.4.1");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "Waiting for user configuration (timeout: %d ms)...", PROVISIONING_TIMEOUT_MS);

    // ========== 显示OLED配网界面 ==========
    display_provisioning_interface(ap_ssid, PROVISIONING_PASSWORD, "192.168.4.1");

    // 等待配网完成或超时
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                          WIFI_PROV_DONE_BIT,
                                          pdFALSE,
                                          pdFALSE,
                                          pdMS_TO_TICKS(PROVISIONING_TIMEOUT_MS));

    // 停止HTTP服务器
    if (s_provisioning_server) {
        httpd_stop(s_provisioning_server);
        s_provisioning_server = NULL;
    }

    if (bits & WIFI_PROV_DONE_BIT) {
        ESP_LOGI(TAG, "✅ Provisioning completed successfully!");
        s_is_provisioning_mode = false;  // 清除配网模式标志
        vEventGroupDelete(s_wifi_event_group);
        return ESP_OK;
    } else {
        ESP_LOGW(TAG, "⏱️ Provisioning timeout");
        s_is_provisioning_mode = false;  // 清除配网模式标志
        vEventGroupDelete(s_wifi_event_group);
        return ESP_ERR_TIMEOUT;
    }
}

/**
 * @brief 检查并处理按键触发配网
 *
 * 检测Boot按键（GPIO 9）长按5秒，清除WiFi配置并重启
 */
void wifi_provisioning_check_button(void) {
    // Boot按键连接到GPIO 9，按下时为低电平
    #define BOOT_BUTTON_GPIO 9
    #define BUTTON_HOLD_TIME_MS 5000  // 长按5秒
    #define CHECK_INTERVAL_MS 100     // 每100ms检查一次

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "按键检测已启动");
    ESP_LOGI(TAG, "长按Boot按键5秒可清除WiFi配置并重新配网");
    ESP_LOGI(TAG, "========================================");

    int press_count = 0;
    const int max_count = BUTTON_HOLD_TIME_MS / CHECK_INTERVAL_MS;

    while (1) {
        // 读取按键状态（0表示按下）
        int button_level = gpio_get_level(BOOT_BUTTON_GPIO);

        if (button_level == 0) {
            // 按键被按下
            press_count++;

            // 每秒提示一次
            if (press_count % 10 == 0) {
                int seconds = (press_count * CHECK_INTERVAL_MS) / 1000;
                ESP_LOGI(TAG, "检测到按键按下... %d/%d秒", seconds, BUTTON_HOLD_TIME_MS / 1000);
            }

            // 达到长按时间
            if (press_count >= max_count) {
                ESP_LOGI(TAG, "========================================");
                ESP_LOGW(TAG, "检测到按键长按5秒！");
                ESP_LOGW(TAG, "正在清除WiFi配置...");
                ESP_LOGI(TAG, "========================================");

                // 清除WiFi配置
                esp_err_t err = wifi_provisioning_clear_config();
                if (err == ESP_OK) {
                    ESP_LOGI(TAG, "✅ WiFi配置已清除");
                    ESP_LOGI(TAG, "设备即将重启进入配网模式...");
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    esp_restart();
                } else {
                    ESP_LOGE(TAG, "❌ 清除配置失败: %s", esp_err_to_name(err));
                }

                // 重置计数
                press_count = 0;
            }
        } else {
            // 按键释放，重置计数
            if (press_count > 0) {
                ESP_LOGI(TAG, "按键释放，重置计数");
                press_count = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(CHECK_INTERVAL_MS));
    }
}
