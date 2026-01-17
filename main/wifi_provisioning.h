#ifndef WIFI_PROVISIONING_H
#define WIFI_PROVISIONING_H

#include <stdbool.h>
#include "esp_err.h"

/**
 * @brief 检查NVS中是否已保存WiFi凭据
 * @return true 如果已保存WiFi凭据
 * @return false 如果未保存WiFi凭据
 */
bool wifi_provisioning_has_config(void);

/**
 * @brief 检查是否处于配网模式
 * @return true 如果当前处于配网模式
 * @return false 如果当前不处于配网模式
 */
bool wifi_provisioning_is_active(void);

/**
 * @brief 从NVS加载WiFi凭据
 * @param ssid 输出参数，存储SSID
 * @param password 输出参数，存储密码
 * @param ssid_buf_size SSID缓冲区大小
 * @param password_buf_size 密码缓冲区大小
 * @return ESP_OK 成功加载
 * @return ESP_ERR_NOT_FOUND 未找到WiFi凭据
 */
esp_err_t wifi_provisioning_load_config(char *ssid, char *password,
                                        size_t ssid_buf_size, size_t password_buf_size);

/**
 * @brief 保存WiFi凭据到NVS
 * @param ssid WiFi SSID
 * @param password WiFi密码
 * @return ESP_OK 成功保存
 */
esp_err_t wifi_provisioning_save_config(const char *ssid, const char *password);

/**
 * @brief 启动WiFi配网模式（SoftAP + HTTP服务器）
 *
 * 该函数会：
 * 1. 启动SoftAP热点（名为"ESP32-Setup_xxxxx"）
 * 2. 启动DNS服务器（ captive portal）
 * 3. 启动HTTP服务器提供配置页面
 * 4. 等待用户完成配置（阻塞直到成功或超时）
 *
 * @return ESP_OK 配网成功
 * @return ESP_FAIL 配网失败或超时
 */
esp_err_t wifi_provisioning_start(void);

/**
 * @brief 清除已保存的WiFi凭据
 * @return ESP_OK 成功清除
 */
esp_err_t wifi_provisioning_clear_config(void);

/**
 * @brief 检查并处理按键触发配网
 *
 * 该函数会检查Boot按键（GPIO 9）是否被长按5秒
 * 如果是，则清除WiFi配置并重启进入配网模式
 *
 * @note 应在启动时调用，延迟2秒后开始检测
 */
void wifi_provisioning_check_button(void);

#endif // WIFI_PROVISIONING_H
