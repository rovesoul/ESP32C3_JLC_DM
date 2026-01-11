#include "simple_wifi_sta.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "lwip/inet.h"
#include "NtcAdcScreenDHT22.h"

// 请根据你家路由器修改下面两个宏：SSID 和 PASSWORD
// 把这两个改成你的 WiFi 名称和密码后再编译烧录
#define DEFAULT_WIFI_SSID           "dm2G"
#define DEFAULT_WIFI_PASSWORD       "88888888dmdmdm"

// 日志 TAG，用于在串口输出中区分本模块的日志
static const char *TAG = "wifi";



/** 事件回调函数
 * @param arg   用户传递的参数
 * @param event_base    事件类别
 * @param event_id      事件ID
 * @param event_data    事件携带的数据
 * @return 无
*/
/*
 * 嵌入资源说明：
 * - CMake 中使用 `EMBED_TXTFILES "index.html"` 会在链接时生成符号：
 *   `_binary_index_html_start` 和 `_binary_index_html_end`，用来指向嵌入数据。
 * - 下面两个 extern 声明用于访问嵌入的 index.html 内容。
 */
extern const unsigned char _binary_index_html_start[] asm("_binary_index_html_start");
extern const unsigned char _binary_index_html_end[]   asm("_binary_index_html_end");

// HTTP server 句柄和启动标志，确保只启动一次
static httpd_handle_t server = NULL;
static bool server_started = false;

// PID 参数全局变量
extern float PID_KP;
extern float PID_KI;
extern float PID_KD;
extern float TARGET_TEMP;
extern float ntcTemp;
extern float dhtTemp;
extern float dhtHumidity;
extern bool is_OPEN;

// 声明 heater_pid 以访问主代码中的 PID 控制器
extern pid_controller_t heater_pid;

/*
 * HTTP GET 处理函数：把嵌入的 index.html 返回给浏览器
 * - 使用 httpd_resp_set_type 设置 Content-Type
 * - 使用 httpd_resp_send 发送整个文件内容
 */
static esp_err_t index_get_handler(httpd_req_t *req)
{
    const unsigned char *data = _binary_index_html_start;
    size_t len = (size_t)(_binary_index_html_end - _binary_index_html_start);
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, (const char *)data, len);
    return ESP_OK;
}

/*
 * HTTP GET 处理函数：返回当前 PID 参数和循环计数
 */
static esp_err_t values_get_handler(httpd_req_t *req)
{
    char response[256];
    float pwm_percent = (heater_pid.pwm_duty * 100.0f) / 10000;
    int len = snprintf(response, sizeof(response),
        "{\"P\":%.2f,\"I\":%.2f,\"D\":%.2f,\"TARGET_TEMP\":%.2f,\"ntcTemp\":%.2f,\"pwmPercent\":%.2f,\"dhtTemp\":%.2f,\"dhtHumidity\":%.2f}",
        PID_KP, PID_KI, PID_KD, TARGET_TEMP, ntcTemp, pwm_percent, dhtTemp, dhtHumidity);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, len);
    return ESP_OK;
}

/*
 * 保存 PID 参数到 NVS
 */
static esp_err_t save_pid_to_nvs(float P, float I, float D, float target_temp)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    // 打开 NVS 命名空间
    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // 将三个 float 打包为数组保存
    float values[4] = {P, I, D, target_temp};

    // 写入 P、I、D 值（使用 blob 方式）
    err = nvs_set_blob(nvs_handle, "pid_values", values, sizeof(values));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving values: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // 提交更改到 Flash
    err = nvs_commit(nvs_handle);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Saved PID and TARGET_TEMP to NVS: P=%.2f, I=%.2f, D=%.2f, TARGET_TEMP=%.2f", P, I, D, target_temp);
    } else {
        ESP_LOGE(TAG, "Error committing NVS changes: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

/*
 * 从 NVS 加载 PID 参数
 */
static esp_err_t load_pid_from_nvs(void)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found, using default values");
        return err;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // 读取 P、I、D 值（使用 blob 方式）
    float values[4];
    size_t required_size = sizeof(values);

    err = nvs_get_blob(nvs_handle, "pid_values", values, &required_size);
    if (err == ESP_OK) {
        PID_KP = values[0];
        PID_KI = values[1];
        PID_KD = values[2];
        TARGET_TEMP = values[3];
        ESP_LOGI(TAG, "Loaded PID and TARGET_TEMP from NVS: P=%.2f, I=%.2f, D=%.2f, TARGET_TEMP=%.2f", PID_KP, PID_KI, PID_KD, TARGET_TEMP);
    } else {
        ESP_LOGI(TAG, "No PID values found in NVS, using defaults");
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

/*
 * HTTP POST 处理函数：接收并设置 PID 参数
 */
static esp_err_t config_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char buf[160];
    int received = 0;

    // 读取请求体
    while (cur_len < total_len) {
        int read_len = httpd_req_recv(req, buf + cur_len, sizeof(buf) - cur_len - 1);
        if (read_len <= 0) {
            if (read_len == HTTPD_SOCK_ERR_TIMEOUT) {
                continue;
            }
            return ESP_FAIL;
        }
        cur_len += read_len;
    }
    buf[cur_len] = '\0';

    // 解析 JSON 并设置值
    float P, I, D, target_temp;
    if (sscanf(buf, "{\"P\":%f,\"I\":%f,\"D\":%f,\"TARGET_TEMP\":%f}", &P, &I, &D, &target_temp) == 4) {
        PID_KP = P;
        PID_KI = I;
        PID_KD = D;
        TARGET_TEMP = target_temp;

        // 更新 PID 控制器的参数
        heater_pid.kp = PID_KP;
        heater_pid.ki = PID_KI;
        heater_pid.kd = PID_KD;
        heater_pid.target_temp = TARGET_TEMP;

        // 保存到 NVS
        esp_err_t err = save_pid_to_nvs(PID_KP, PID_KI, PID_KD, TARGET_TEMP);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Updated PID and TARGET_TEMP: P=%.2f, I=%.2f, D=%.2f, TARGET_TEMP=%.2f", PID_KP, PID_KI, PID_KD, TARGET_TEMP);
            httpd_resp_send(req, "{\"status\":\"ok\"}", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        } else {
            httpd_resp_send(req, "{\"status\":\"error\"}", HTTPD_RESP_USE_STRLEN);
            return ESP_FAIL;
        }
    }

    httpd_resp_send(req, "{\"status\":\"error\"}", HTTPD_RESP_USE_STRLEN);
    return ESP_FAIL;
}



/*
 * 保存 is_OPEN 状态到 NVS
 */
static esp_err_t save_toggle_state_to_nvs(bool state) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    err = nvs_set_u8(nvs_handle, "toggle_state", state ? 1 : 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving toggle state: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing toggle state: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

/*
 * 从 NVS 加载 is_OPEN 状态
 */
static esp_err_t load_toggle_state_from_nvs(bool *state) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found, using default toggle state");
        *state = false;
        return ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    uint8_t saved_state = 0;
    err = nvs_get_u8(nvs_handle, "toggle_state", &saved_state);
    if (err == ESP_OK) {
        *state = saved_state ? true : false;
        ESP_LOGI(TAG, "Loaded toggle state from NVS: %s", *state ? "OPEN" : "CLOSED");
    } else {
        ESP_LOGI(TAG, "No toggle state found in NVS, using default");
        *state = false;
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

/*
 * HTTP POST 处理函数：切换状态
 */
static esp_err_t toggle_post_handler(httpd_req_t *req) {
    is_OPEN = !is_OPEN; // 切换状态
    save_toggle_state_to_nvs(is_OPEN); // 保存状态到 NVS

    const char *response = is_OPEN ? "{\"status\":\"OPEN\"}" : "{\"status\":\"CLOSED\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "状态切换为: %s", is_OPEN ? "OPEN" : "CLOSED");
    return ESP_OK;
}

// HTTP GET 处理函数：返回当前状态天0-esw0-mnnnnnnnnnnnnnnnnnnnnnn
static esp_err_t toggle_get_handler(httpd_req_t *req) {
    const char *response = is_OPEN ? "{\"status\":\"OPEN\"}" : "{\"status\":\"CLOSED\"}";
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "获取状态: %s", is_OPEN ? "OPEN" : "CLOSED");
    return ESP_OK;
}

/*
 * 注册切换状态的 URI
 */
void register_toggle_uri(httpd_handle_t server) {
    httpd_uri_t uri_toggle_post = {
        .uri       = "/toggle",
        .method    = HTTP_POST,
        .handler   = toggle_post_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &uri_toggle_post);

    httpd_uri_t uri_toggle_get = {
        .uri       = "/toggle",
        .method    = HTTP_GET,
        .handler   = toggle_get_handler,
        .user_ctx  = NULL
    };
    httpd_register_uri_handler(server, &uri_toggle_get);
}

/*
 * 事件回调函数：处理 WiFi 相关事件和 IP 获取事件
 * - WIFI_EVENT_*: 跟 WiFi 连接状态相关（启动、连接、断开）
 * - IP_EVENT_STA_GOT_IP: 只有获取到路由器分配的 IP，才认为真正连上网络
 *
 * 在获取到 IP 后启动 HTTP 服务并打印设备 IP
 */
static void event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{   
    if(event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_STA_START:      //WIFI以STA模式启动后触发此事件
            // STA 启动后主动连接已配置的 AP
            esp_wifi_connect();
            break;
        case WIFI_EVENT_STA_CONNECTED:  //WIFI连上路由器后，触发此事件
            // 已与路由器建立链路（尚未获取 IP）
            ESP_LOGI(TAG, "connected to AP");
            break;
        case WIFI_EVENT_STA_DISCONNECTED:   //WIFI从路由器断开连接后触发此事件
            // 断开后启动重连（简单重连策略）
            esp_wifi_connect();
            ESP_LOGI(TAG,"connect to the AP fail,retry now");
            break;
        default:
            break;
        }
    }
    if(event_base == IP_EVENT)                  //IP相关事件
    {
        switch(event_id)
        {
            case IP_EVENT_STA_GOT_IP:           // 取得 DHCP 分配的 IP
                {
                    // 从事件数据结构中提取 IP 信息并打印（更易读）
                    ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                    char ip_str[16];
                    inet_ntoa_r(event->ip_info.ip, ip_str, sizeof(ip_str));
                    ESP_LOGI(TAG, "got ip: %s", ip_str);

                    // 仅在尚未启动 HTTP 服务时启动一次
                    if (!server_started) {
                        httpd_config_t config = HTTPD_DEFAULT_CONFIG();
                        if (httpd_start(&server, &config) == ESP_OK) {
                            httpd_uri_t uri_get = {
                                .uri       = "/",
                                .method    = HTTP_GET,
                                .handler   = index_get_handler,
                                .user_ctx  = NULL
                            };
                            httpd_register_uri_handler(server, &uri_get);

                            httpd_uri_t uri_values = {
                                .uri       = "/values",
                                .method    = HTTP_GET,
                                .handler   = values_get_handler,
                                .user_ctx  = NULL
                            };
                            httpd_register_uri_handler(server, &uri_values);

                            httpd_uri_t uri_config = {
                                .uri       = "/config",
                                .method    = HTTP_POST,
                                .handler   = config_post_handler,
                                .user_ctx  = NULL
                            };
                            httpd_register_uri_handler(server, &uri_config);

                            // 注册新的 /toggle URI
                            register_toggle_uri(server);

                            server_started = true; // 标记已启动
                            ESP_LOGI(TAG, "HTTP server started");
                        } else {
                            ESP_LOGW(TAG, "Failed to start HTTP server");
                        }
                    }
                }
                break;
        }
    }
}


// 在 WiFi 初始化时重置 toggle_state 为 0
static void reset_toggle_state_on_boot(void) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_u8(nvs_handle, "toggle_state", 0); // 设置为 0
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
        ESP_LOGI(TAG, "toggle_state 已重置为 0");
    } else {
        ESP_LOGE(TAG, "无法打开 NVS: %s", esp_err_to_name(err));
    }
}

//WIFI STA初始化
esp_err_t wifi_sta_init(void)
{
    ESP_ERROR_CHECK(esp_netif_init());  //用于初始化tcpip协议栈
    ESP_ERROR_CHECK(esp_event_loop_create_default());       //创建一个默认系统事件调度循环，之后可以注册回调函数来处理系统的一些事件
    esp_netif_create_default_wifi_sta();    //使用默认配置创建STA对象

    // 初始化WIFI
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 注册事件
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT,ESP_EVENT_ANY_ID,&event_handler,NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT,IP_EVENT_STA_GOT_IP,&event_handler,NULL));

    // WIFI配置
    wifi_config_t wifi_config =
    {
        .sta =
        {
            .ssid = DEFAULT_WIFI_SSID,              //WIFI的SSID
            .password = DEFAULT_WIFI_PASSWORD,      //WIFI密码
	        .threshold.authmode = WIFI_AUTH_WPA2_PSK,   //加密方式

            .pmf_cfg =
            {
                .capable = true,
                .required = false
            },
        },
    };

    // 启动WIFI
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );         //设置工作模式为STA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );   //设置wifi配置
    ESP_ERROR_CHECK(esp_wifi_start() );                         //启动WIFI

    reset_toggle_state_on_boot(); // 上电时重置 toggle_state
    load_pid_from_nvs();
    load_toggle_state_from_nvs(&is_OPEN); // 加载开关状态

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    return ESP_OK;
}