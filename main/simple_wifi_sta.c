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
#include "esp_timer.h"
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
extern float FAN_SPEED_PERCENT;
extern float TIMER_HOURS_CONFIG;
extern bool timer_is_running;
extern int64_t timer_start_time_ms;
extern int64_t system_start_time_ms;

// 函数声明
void set_fan_pwm(float percent);
void create_system_timer(float hours);
void start_system_timer(void);
void stop_system_timer(void);

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
    char response[500];
    float pwm_percent = (heater_pid.pwm_duty * 100.0f) / 10000;

    // 计算定时器剩余秒数
    int timer_remaining_seconds = 0;
    if (timer_is_running && TIMER_HOURS_CONFIG > 0.0f) {
        int64_t current_time_ms = esp_timer_get_time() / 1000;
        int64_t elapsed_ms = current_time_ms - timer_start_time_ms;
        int total_seconds = (int)(TIMER_HOURS_CONFIG * 3600.0f);
        timer_remaining_seconds = total_seconds - (int)(elapsed_ms / 1000);
        if (timer_remaining_seconds < 0) timer_remaining_seconds = 0;
    }

    // 计算系统运行时间（秒）
    int system_running_seconds = 0;
    if (is_OPEN && system_start_time_ms > 0) {
        int64_t current_time_ms = esp_timer_get_time() / 1000;
        system_running_seconds = (int)((current_time_ms - system_start_time_ms) / 1000);
        if (system_running_seconds < 0) system_running_seconds = 0;
    }

    int len = snprintf(response, sizeof(response),
        "{\"P\":%.2f,\"I\":%.2f,\"D\":%.2f,\"TARGET_TEMP\":%.2f,\"ntcTemp\":%.2f,\"pwmPercent\":%.2f,\"dhtTemp\":%.2f,\"dhtHumidity\":%.2f,\"fanSpeed\":%.1f,\"timerHours\":%.1f,\"timerRemaining\":%d,\"systemRunningSeconds\":%d}",
        PID_KP, PID_KI, PID_KD, TARGET_TEMP, ntcTemp, pwm_percent, dhtTemp, dhtHumidity, FAN_SPEED_PERCENT, TIMER_HOURS_CONFIG, timer_remaining_seconds, system_running_seconds);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, len);
    return ESP_OK;
}

/*
 * 统一保存配置到 NVS（一次性保存所有配置）
 * @param save_pid: 是否保存 PID 参数
 * @param save_fan: 是否保存风扇转速
 * @param save_toggle: 是否保存开关状态
 * @param save_timer: 是否保存定时器配置
 * @param P, I, D, TARGET_TEMP: PID 参数（当 save_pid=true 时有效）
 * @param fan_speed: 风扇转速（当 save_fan=true 时有效）
 * @param toggle_state: 开关状态（当 save_toggle=true 时有效）
 * @param timer_hours: 定时器时长（当 save_timer=true 时有效）
 */
static esp_err_t save_config_to_nvs(bool save_pid, bool save_fan, bool save_toggle, bool save_timer,
                                     float P, float I, float D, float target_temp,
                                     float fan_speed, bool toggle_state, float timer_hours)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // 保存 PID 参数
    if (save_pid) {
        float pid_values[4] = {P, I, D, target_temp};
        err = nvs_set_blob(nvs_handle, "pid_values", pid_values, sizeof(pid_values));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Saved PID to NVS: P=%.2f, I=%.2f, D=%.2f, TARGET=%.2f", P, I, D, target_temp);
        } else {
            ESP_LOGE(TAG, "Error saving PID: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }
    }

    // 保存风扇转速
    if (save_fan) {
        ESP_LOGI(TAG, "=== 准备保存风扇转速到 NVS: %.1f%% ===", fan_speed);
        err = nvs_set_blob(nvs_handle, "fan_speed", &fan_speed, sizeof(fan_speed));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "✅ 风扇转速已保存到 NVS: %.1f%%", fan_speed);
        } else {
            ESP_LOGE(TAG, "❌ 保存风扇转速失败: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }
    }

    // 保存开关状态
    if (save_toggle) {
        err = nvs_set_u8(nvs_handle, "toggle_state", toggle_state ? 1 : 0);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Saved toggle state to NVS: %s", toggle_state ? "OPEN" : "CLOSED");
        } else {
            ESP_LOGE(TAG, "Error saving toggle state: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }
    }

    // 保存定时器配置
    if (save_timer) {
        err = nvs_set_blob(nvs_handle, "timer_hours", &timer_hours, sizeof(timer_hours));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "✅ 定时器配置已保存到 NVS: %.1f小时", timer_hours);
        } else {
            ESP_LOGE(TAG, "❌ 保存定时器配置失败: %s", esp_err_to_name(err));
            nvs_close(nvs_handle);
            return err;
        }
    }

    // 提交所有更改到 Flash
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

/*
 * 统一从 NVS 加载配置（一次性加载所有配置）
 * @param load_pid: 是否加载 PID 参数
 * @param load_fan: 是否加载风扇转速
 * @param load_toggle: 是否加载开关状态
 * @param load_timer: 是否加载定时器配置
 * @param state_out: 开关状态输出指针（当 load_toggle=true 时有效）
 * @param apply_fan_pwm: 是否立即应用风扇PWM（通常上电时应为false）
 */
static esp_err_t load_config_from_nvs(bool load_pid, bool load_fan, bool load_toggle, bool load_timer,
                                       bool *state_out, bool apply_fan_pwm)
{
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "NVS namespace not found, using defaults");
        if (load_toggle && state_out) *state_out = false;
        if (load_fan) FAN_SPEED_PERCENT = 100.0f;
        return ESP_OK;
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS namespace: %s", esp_err_to_name(err));
        return err;
    }

    // 加载 PID 参数
    if (load_pid) {
        float pid_values[4];
        size_t required_size = sizeof(pid_values);
        err = nvs_get_blob(nvs_handle, "pid_values", pid_values, &required_size);
        if (err == ESP_OK) {
            PID_KP = pid_values[0];
            PID_KI = pid_values[1];
            PID_KD = pid_values[2];
            TARGET_TEMP = pid_values[3];
            ESP_LOGI(TAG, "Loaded PID from NVS: P=%.2f, I=%.2f, D=%.2f, TARGET=%.2f",
                     PID_KP, PID_KI, PID_KD, TARGET_TEMP);
        } else {
            ESP_LOGI(TAG, "No PID values found in NVS, using defaults");
        }
    }

    // 加载风扇转速
    if (load_fan) {
        float fan_speed;
        size_t required_size = sizeof(fan_speed);
        ESP_LOGI(TAG, "=== 准备从 NVS 读取风扇转速 ===");
        err = nvs_get_blob(nvs_handle, "fan_speed", &fan_speed, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "✅ 从 NVS 读取到风扇转速: %.1f%%", fan_speed);
            FAN_SPEED_PERCENT = fan_speed;
            if (apply_fan_pwm) {
                set_fan_pwm(fan_speed);
            }
            ESP_LOGI(TAG, "风扇转速已更新: %.1f%% %s",
                     FAN_SPEED_PERCENT, apply_fan_pwm ? "(已应用PWM)" : "(未应用PWM)");
        } else {
            ESP_LOGW(TAG, "⚠️ NVS 中未找到风扇转速，使用默认值 100%%");
            FAN_SPEED_PERCENT = 100.0f;
        }
    }

    // 加载定时器配置
    if (load_timer) {
        float timer_hours;
        size_t required_size = sizeof(timer_hours);
        err = nvs_get_blob(nvs_handle, "timer_hours", &timer_hours, &required_size);
        if (err == ESP_OK) {
            TIMER_HOURS_CONFIG = timer_hours;
            ESP_LOGI(TAG, "✅ 从 NVS 读取到定时器配置: %.1f小时", TIMER_HOURS_CONFIG);
            // 创建定时器（但不启动）
            create_system_timer(TIMER_HOURS_CONFIG);
        } else {
            ESP_LOGW(TAG, "⚠️ NVS 中未找到定时器配置，使用默认值 0小时");
            TIMER_HOURS_CONFIG = 0.0f;
        }
    }

    // 加载开关状态
    if (load_toggle && state_out) {
        uint8_t saved_state = 0;
        err = nvs_get_u8(nvs_handle, "toggle_state", &saved_state);
        if (err == ESP_OK) {
            *state_out = saved_state ? true : false;
            ESP_LOGI(TAG, "Loaded toggle state from NVS: %s", *state_out ? "OPEN" : "CLOSED");
        } else {
            ESP_LOGI(TAG, "No toggle state found in NVS, using default");
            *state_out = false;
        }
    }

    nvs_close(nvs_handle);
    return ESP_OK;
}

/*
 * HTTP POST 处理函数：接收并设置 PID 参数和风扇转速
 */
static esp_err_t config_post_handler(httpd_req_t *req)
{
    int total_len = req->content_len;
    int cur_len = 0;
    char buf[250];

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

    ESP_LOGI(TAG, "=== 收到前端配置数据: %s ===", buf);

    // 解析 JSON 并设置值 (包含风扇转速和定时器)
    float P, I, D, target_temp, fan_speed, timer_hours;
    int parsed = sscanf(buf, "{\"P\":%f,\"I\":%f,\"D\":%f,\"TARGET_TEMP\":%f,\"fanSpeed\":%f,\"timerHours\":%f}",
                        &P, &I, &D, &target_temp, &fan_speed, &timer_hours);

    ESP_LOGI(TAG, "=== JSON解析结果: parsed=%d, P=%.2f, I=%.2f, D=%.2f, TARGET=%.2f, FAN=%.1f, TIMER=%.1f ===",
             parsed, P, I, D, target_temp, fan_speed, timer_hours);

    if (parsed == 6) {  // 6个参数都成功解析
        PID_KP = P;
        PID_KI = I;
        PID_KD = D;
        TARGET_TEMP = target_temp;

        // 更新 PID 控制器的参数
        heater_pid.kp = PID_KP;
        heater_pid.ki = PID_KI;
        heater_pid.kd = PID_KD;
        heater_pid.target_temp = TARGET_TEMP;

        // 限制风扇转速范围
        if (fan_speed < 0.0f) fan_speed = 0.0f;
        if (fan_speed > 100.0f) fan_speed = 100.0f;

        FAN_SPEED_PERCENT = fan_speed;
        set_fan_pwm(fan_speed);

        // 更新定时器配置
        TIMER_HOURS_CONFIG = timer_hours;
        create_system_timer(timer_hours);
        ESP_LOGI(TAG, "✅ 定时器配置已更新: %.1f小时", timer_hours);

        // 保存到 NVS（统一保存 PID + 风扇 + 定时器）
        esp_err_t err = save_config_to_nvs(true, true, false, true,
                                          PID_KP, PID_KI, PID_KD, TARGET_TEMP,
                                          fan_speed, false, timer_hours);

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Updated config: P=%.2f, I=%.2f, D=%.2f, TARGET_TEMP=%.2f, fanSpeed=%.1f%%, timerHours=%.1f",
                     PID_KP, PID_KI, PID_KD, TARGET_TEMP, fan_speed, timer_hours);
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
 * HTTP POST 处理函数：切换状态
 */
static esp_err_t toggle_post_handler(httpd_req_t *req) {
    is_OPEN = !is_OPEN; // 切换状态

    // 根据系统状态启动或停止定时器
    if (is_OPEN) {
        // 系统启动，如果设置了定时器则启动
        if (TIMER_HOURS_CONFIG > 0.0f) {
            start_system_timer();
            ESP_LOGI(TAG, "⏱️ 系统启动，定时器已启动: %.1f小时", TIMER_HOURS_CONFIG);
        } else {
            ESP_LOGI(TAG, "✅ 系统启动，未设置定时器");
        }
        // 记录系统启动时间
        system_start_time_ms = esp_timer_get_time() / 1000;
        ESP_LOGI(TAG, "⏰ 系统运行时间已开始记录");
    } else {
        // 系统关闭，停止定时器
        stop_system_timer();
        system_start_time_ms = 0;  // 重置运行时间
        ESP_LOGI(TAG, "⏹️ 系统关闭，定时器已停止，运行时间已重置");
    }

    save_config_to_nvs(false, false, true, false, 0, 0, 0, 0, 0, is_OPEN, 0.0f); // 只保存开关状态

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

                            // 注册 /toggle URI
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

    // 上电时强制重置为关闭状态，防止断电恢复后热惯性导致过热
    ESP_LOGI(TAG, "========== 上电初始化开始 ==========");

    is_OPEN = false;  // 强制设置为关闭状态
    ESP_LOGI(TAG, "① 设置 is_OPEN = false");

    save_config_to_nvs(false, false, true, false, 0, 0, 0, 0, 0, is_OPEN, 0.0f);  // 写入 NVS
    ESP_LOGI(TAG, "② 已保存开关状态到 NVS（不保存风扇转速）");


    // 统一加载所有配置（PID + 风扇 + 定时器，不立即应用PWM）
    ESP_LOGI(TAG, "④ 准备从 NVS 加载配置...");
    load_config_from_nvs(true, true, false, true, NULL, false);

    ESP_LOGI(TAG, "========== 上电初始化完成，最终风扇转速: %.1f%%, 定时器: %.1f小时 ==========",
             FAN_SPEED_PERCENT, TIMER_HOURS_CONFIG);

    ESP_LOGI(TAG, "上电初始化完成: is_OPEN=false (需手动启动), 风扇已关闭");

    ESP_LOGI(TAG, "wifi_init_sta finished.");
    return ESP_OK;
}