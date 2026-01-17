#ifndef _WIFI_MANAGER_H_
#define _WIFI_MANAGER_H_
#include "esp_err.h"

//WIFI STA初始化
esp_err_t wifi_sta_init(void);

// 获取WiFi IP地址字符串
const char* get_wifi_ip_address(void);

#endif
