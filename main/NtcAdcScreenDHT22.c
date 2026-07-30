/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * 【√】显示adc、dht
 * 【√】增加pid控温
 * 【√】温度曲线，挪到左边
 * 【√】温度码表，优化pid
 * 【√】优化代码，使pid生效 
 * 【√】网页配置界面、曲线、开关按钮
 * 【√】联网设置温度
 * 【√】增加定时功能
 * 【√】增加风扇控制
 * 【√】增加风扇实际速度显示
 * 【√】配置联网模式
 * 【√】配网模式的oled显示
 * 【√】修改仪表盘为半圆，并且显示
 * 【√】倒计时结束，不关机bug，已修改
 * 【】关机一定时间后不断wifi待机
 * 【】阶段优化冗余代码
 * 【】拆分代码
 * 【】ui风格切换（黑金、夜晚、白天、目前的）
 * 【】通风换气
 * 【】联网mqtt显示数main/NtcAdcScreenDHT22.c据
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "dht22.h"
#include "driver/ledc.h"
#include "ntc.h"
#include "esp_err.h"
#include "OLED.h"
#include "simple_wifi_sta.h"
#include "wifi_provisioning.h"
#include "ina226_monitor.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <inttypes.h>

#define TAG       "main"
#define ntcTAG    "ntc"
#define dhtTAG    "dht22"
#define screenTAG "screen"
#define ledTAG    "led"
#define pidTAG    "PID"



// ========== 信号量 ==========
SemaphoreHandle_t dht22_mutex;
SemaphoreHandle_t oled_mutex;

// ========== 呼吸灯配置 (GPIO 8) ==========
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO  (2)
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY  (1000)

// ========== 加热片PWM配置 (GPIO 10) ==========
#define HEATER_LEDC_TIMER    LEDC_TIMER_1
#define HEATER_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define HEATER_OUTPUT_IO     (10)
#define HEATER_LEDC_CHANNEL  LEDC_CHANNEL_1
#define HEATER_DUTY_RES      LEDC_TIMER_10_BIT  // 调整为10位分辨率
#define HEATER_FREQUENCY     (500)              // 调整频率为500Hz

// ========== 风扇PWM配置 (GPIO 6) ==========
#define FAN_LEDC_TIMER       LEDC_TIMER_2
#define FAN_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define FAN_PWM_IO           GPIO_NUM_6
#define FAN_LEDC_CHANNEL     LEDC_CHANNEL_2
#define FAN_DUTY_RES         LEDC_TIMER_10_BIT  // 10位分辨率 (0-1023)
#define FAN_FREQUENCY        (500)            // 500Hz PWM

// ========== 风扇控制模式选择 ==========
#define FAN_USE_GPIO_OUTPUT  0  // 0=PWM控制, 1=简单GPIO开关(用于测试MOSFET)


// ========== PID控制参数 ==========


float TARGET_TEMP  = 35.0f;    //目标温度
float PID_KP  = 200.0f;   // 增大比例增益
float PID_KI  = 180.0f;    // 增大积分增益
float PID_KD  = 10.0f;    // 减小微分增益
#define PID_INTERVAL_MS  200
#define MAX_TEMP_LIMIT   95.0f
#define MIN_TEMP_LIMIT   0.0f
#define MAX_PWM_DUTY     10000    // 提高最大占空比
#define MIN_PWM_DUTY     0

// ========== 显示模式选择 ==========
#define DISPLAY_MODE    2    // 0=曲线+文字  1=仪表盘+文字  2=半圆仪表盘+文字

// ========== OLED配置 ==========
#define OLED_I2C    I2C_NUM_0
#define OLED_SCL    GPIO_NUM_5
#define OLED_SDA    GPIO_NUM_4
#define OLED_ADD    0x78
#define OLED_SPEED  400000

// ========== 风扇转速配置 ==========
float FAN_SPEED_PERCENT = 100.0f;    // 风扇转速百分比 (0-100)
#define MAX_FAN_DUTY  10000           // 风扇最大占空比
float FAN_ACTUAL_PWM = 0.0f;         // 风扇实际PWM输出百分比 (0-100)
bool FAN_IS_RUNNING = false;         // 风扇运行状态 (true=运行, false=停止)

// ========== 温度曲线配置 ==========
#define CURVE_START_X   0      // 曲线起始X坐标（右半屏）
#define CURVE_WIDTH     60      // 曲线宽度
#define CURVE_HEIGHT    64      // 曲线高度
#define CURVE_POINTS    56      // 曲线数据点数量
#define TEXT_START_X    64      //文字开始位置
#define TEMP_MIN        10.0f   // 曲线显示最低温度
#define TEMP_MAX        TARGET_TEMP+10.0f   // 曲线显示最高温度

// ========== 仪表盘配置 ==========
#define GAUGE_CENTER_X      30      // 仪表盘中心X坐标（左侧区域中心）
#define GAUGE_CENTER_Y      30      // 仪表盘中心Y坐标（屏幕中心）
#define GAUGE_RADIUS        30      // 仪表盘外圈半径
#define GAUGE_INNER_RADIUS  24      // 仪表盘内圈半径（用于刻度）
#define GAUGE_POINTER_LEN   20      // 指针长度
#define GAUGE_MIN_TEMP      0.0f    // 仪表盘最低温度
#define GAUGE_MAX_TEMP      100.0f  // 仪表盘最高温度
#define GAUGE_START_ANGLE   135     // 起始角度（左下，0℃位置）
#define GAUGE_END_ANGLE     405     // 结束角度（右下，100℃位置）共270度



// ========== PID结构体 ==========
typedef struct {
    float target_temp;
    float kp, ki, kd;
    float integral;
    float last_error;
    float filtered_derivative;  // 新增：滤波后的微分值
    float output;
    uint32_t pwm_duty;
} pid_controller_t;

// ========== 全局变量 ==========
pid_controller_t heater_pid;
float ntcTemp = 0.0f;
float dhtTemp = 0.0f;
float dhtHumidity = 0.0f;
bool dhtAvailable = false;
bool dhtHasValidReading = false;
uint32_t dhtFailCount = 0;
// 全局变量表示当前状态（红/绿）
bool is_OPEN = false;

// ========== 定时器相关变量 ==========
float TIMER_HOURS_CONFIG = 0.0f;  // 用户配置的定时器时长（小时）
TimerHandle_t timer_handle = NULL;  // FreeRTOS 定时器句柄
int64_t timer_start_time_ms = 0;  // 定时器启动时间（毫秒）
bool timer_is_running = false;  // 定时器是否正在运行

// ========== 系统运行时间跟踪 ==========
int64_t system_start_time_ms = 0;  // 系统启动时间（毫秒）

// ========== 数学辅助宏 ==========
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ========== 温度曲线缓冲区 ==========
static float temp_curve[CURVE_POINTS] = {0};
static uint8_t curve_index = 0;

// ========== 仪表盘绘制函数 ==========

/**
 * @brief 将温度转换为角度（圆形仪表盘）
 * @param temp 温度值
 * @return 对应的角度（度）
 */
float temp_to_angle(float temp)
{
    if (temp < GAUGE_MIN_TEMP) temp = GAUGE_MIN_TEMP;
    if (temp > GAUGE_MAX_TEMP) temp = GAUGE_MAX_TEMP;

    float ratio = (temp - GAUGE_MIN_TEMP) / (GAUGE_MAX_TEMP - GAUGE_MIN_TEMP);
    float angle = GAUGE_START_ANGLE + ratio * (GAUGE_END_ANGLE - GAUGE_START_ANGLE);

    return angle;
}

/**
 * @brief 将数值向上取整到最近的10的倍数
 * @param val 输入值
 * @return 向上取整到10的倍数
 */
int round_up_to_10(float val)
{
    int ival = (int)val;
    return (ival + 9) / 10 * 10;
}

/**
 * @brief 将温度转换为半圆角度
 * @param temp 温度值
 * @param max_temp 最大温度值
 * @return 对应的角度（度），-180°=左侧，-90°=顶部，0°=右侧
 */
float temp_to_semi_angle(float temp, float max_temp)
{
    if (temp < 0) temp = 0;
    if (temp > max_temp) temp = max_temp;

    float ratio = temp / max_temp;
    float angle = -180.0f + ratio * 180.0f;  // -180°~0°

    return angle;
}

/**
 * @brief 绘制仪表盘刻度点
 */
void draw_gauge_ticks(void)
{
    // 每10℃画一个刻度点
    for (int temp = 0; temp <= 100; temp += 10) {
        float angle = temp_to_angle((float)temp);
        float rad = angle * M_PI / 180.0f;
        
        // 外圈刻度点（稍大）
        int16_t x_outer = GAUGE_CENTER_X + (int16_t)((GAUGE_RADIUS-3) * cos(rad));
        int16_t y_outer = GAUGE_CENTER_Y + (int16_t)((GAUGE_RADIUS-3) * sin(rad));
        
        // 画一个小圆点（3x3像素）
        OLED_DrawPoint(x_outer, y_outer);
        OLED_DrawPoint(x_outer + 1, y_outer);
        OLED_DrawPoint(x_outer, y_outer + 1);
        OLED_DrawPoint(x_outer - 1, y_outer);
        OLED_DrawPoint(x_outer, y_outer - 1);
    }
}

/**
 * @brief 绘制目标温度三角形标记（圆形仪表盘）
 * @param target_temp 目标温度
 */
void draw_target_marker(float target_temp)
{
    float angle = temp_to_angle(target_temp);
    float rad = angle * M_PI / 180.0f;

    // 三角形顶点在内
    int16_t x_tip = GAUGE_CENTER_X + (int16_t)((GAUGE_RADIUS -3) * cos(rad));
    int16_t y_tip = GAUGE_CENTER_Y + (int16_t)((GAUGE_RADIUS -3) * sin(rad));

    // 三角形底边两个点（指向圆外）
    float perp_angle1 = angle + 8;// 增大=三角形更宽
    float perp_angle2 = angle - 8;
    float perp_rad1 = perp_angle1 * M_PI / 180.0f;
    float perp_rad2 = perp_angle2 * M_PI / 180.0f;

    int16_t base_dist = GAUGE_RADIUS +4; // 增大=三角形更长
    int16_t x_base1 = GAUGE_CENTER_X + (int16_t)(base_dist * cos(perp_rad1));
    int16_t y_base1 = GAUGE_CENTER_Y + (int16_t)(base_dist * sin(perp_rad1));

    int16_t x_base2 = GAUGE_CENTER_X + (int16_t)(base_dist * cos(perp_rad2));
    int16_t y_base2 = GAUGE_CENTER_Y + (int16_t)(base_dist * sin(perp_rad2));

    // 绘制实心三角形
    OLED_DrawTriangle(x_tip, y_tip, x_base1, y_base1, x_base2, y_base2, OLED_FILLED);
}

/**
 * @brief 绘制半圆目标温度三角形标记
 * @param target_temp 目标温度
 * @param max_temp 最大温度值
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param radius 半径
 */
void draw_semi_target_marker(float target_temp, float max_temp, int16_t center_x, int16_t center_y, int16_t radius)
{
    float angle = temp_to_semi_angle(target_temp, max_temp);
    float rad = angle * M_PI / 180.0f;

    // 三角形顶点在内
    int16_t x_tip = center_x + (int16_t)((radius - 3) * cos(rad));
    int16_t y_tip = center_y + (int16_t)((radius - 3) * sin(rad));

    // 三角形底边两个点（指向圆外）
    float perp_angle1 = angle + 8;
    float perp_angle2 = angle - 8;
    float perp_rad1 = perp_angle1 * M_PI / 180.0f;
    float perp_rad2 = perp_angle2 * M_PI / 180.0f;

    int16_t base_dist = radius + 4;
    int16_t x_base1 = center_x + (int16_t)(base_dist * cos(perp_rad1));
    int16_t y_base1 = center_y + (int16_t)(base_dist * sin(perp_rad1));

    int16_t x_base2 = center_x + (int16_t)(base_dist * cos(perp_rad2));
    int16_t y_base2 = center_y + (int16_t)(base_dist * sin(perp_rad2));

    // 绘制实心三角形
    OLED_DrawTriangle(x_tip, y_tip, x_base1, y_base1, x_base2, y_base2, OLED_FILLED);
}

/**
 * @brief 绘制温度指针（圆形仪表盘）
 * @param current_temp 当前温度
 */
void draw_temperature_pointer(float current_temp)
{
    float angle = temp_to_angle(current_temp);
    float rad = angle * M_PI / 180.0f;

    // 指针从中心指向外圈
    int16_t x_end = GAUGE_CENTER_X + (int16_t)(GAUGE_POINTER_LEN * cos(rad));
    int16_t y_end = GAUGE_CENTER_Y + (int16_t)(GAUGE_POINTER_LEN * sin(rad));

    // 绘制指针线（加粗效果）
    OLED_DrawLine(GAUGE_CENTER_X, GAUGE_CENTER_Y, x_end, y_end);
    OLED_DrawLine(GAUGE_CENTER_X + 1, GAUGE_CENTER_Y, x_end + 1, y_end);
    OLED_DrawLine(GAUGE_CENTER_X, GAUGE_CENTER_Y + 1, x_end, y_end + 1);

    // 中心点（表盘中心装饰）
    OLED_DrawCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, 2, OLED_FILLED);
}

/**
 * @brief 绘制半圆温度指针
 * @param current_temp 当前温度
 * @param max_temp 最大温度值
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param pointer_len 指针长度
 */
void draw_semi_temperature_pointer(float current_temp, float max_temp, int16_t center_x, int16_t center_y, int16_t pointer_len)
{
    float angle = temp_to_semi_angle(current_temp, max_temp);
    float rad = angle * M_PI / 180.0f;

    // 指针从中心指向外圈
    int16_t x_end = center_x + (int16_t)(pointer_len * cos(rad));
    int16_t y_end = center_y + (int16_t)(pointer_len * sin(rad));

    // 绘制指针线（加粗效果）
    OLED_DrawLine(center_x, center_y, x_end, y_end);
    OLED_DrawLine(center_x + 1, center_y, x_end + 1, y_end);
    OLED_DrawLine(center_x, center_y + 1, x_end, y_end + 1);

    // 中心点
    OLED_DrawCircle(center_x, center_y, 2, OLED_FILLED);
}

/**
 * @brief 绘制完整仪表盘（圆形）
 * @param current_temp 当前温度
 * @param target_temp 目标温度
 */
void draw_gauge(float current_temp, float target_temp)
{
    // 1. 绘制外圈圆环
    OLED_DrawCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_RADIUS, OLED_UNFILLED);
    // OLED_DrawCircle(GAUGE_CENTER_X, GAUGE_CENTER_Y, GAUGE_INNER_RADIUS, OLED_UNFILLED);
    // 2. 绘制刻度点
    draw_gauge_ticks();

    // 3. 绘制目标温度三角形
    draw_target_marker(target_temp);

    // 4. 绘制温度指针
    draw_temperature_pointer(current_temp);

    // 5. 在仪表盘下方显示温度数值
    char temp_str[16];
    snprintf(temp_str, sizeof(temp_str), "%.1fC", current_temp);

    // 居中显示温度数值
    uint8_t str_len = strlen(temp_str);
    uint8_t str_width = str_len * 6;  // OLED_6X8字体宽度
    int16_t text_x = GAUGE_CENTER_X - str_width / 2;
    int16_t text_y = GAUGE_CENTER_Y + GAUGE_RADIUS + 6;

    if (text_y < 64 - 8) {  // 确保不超出屏幕
        OLED_ShowString(text_x, text_y, temp_str, OLED_6X8);
    }
}

/**
 * @brief 绘制半圆仪表盘刻度点
 * @param max_temp 最大温度值
 * @param center_x 圆心X坐标
 * @param center_y 圆心Y坐标
 * @param radius 半径
 */
void draw_semi_gauge_ticks(float max_temp, int16_t center_x, int16_t center_y, int16_t radius)
{
    // 计算刻度间隔（每10度一个刻度）
    int tick_interval = 10;
    int num_ticks = (int)max_temp / tick_interval;

    for (int i = 0; i <= num_ticks; i++) {
        float temp = i * tick_interval;
        float angle = temp_to_semi_angle(temp, max_temp);
        float rad = angle * M_PI / 180.0f;

        // 外圈刻度点
        int16_t x_outer = center_x + (int16_t)((radius - 2) * cos(rad));
        int16_t y_outer = center_y + (int16_t)((radius - 2) * sin(rad));

        // 画一个小圆点（3x3像素）
        OLED_DrawPoint(x_outer, y_outer);
        OLED_DrawPoint(x_outer + 1, y_outer);
        OLED_DrawPoint(x_outer, y_outer + 1);
        OLED_DrawPoint(x_outer - 1, y_outer);
        OLED_DrawPoint(x_outer, y_outer - 1);
    }
}

/**
 * @brief 绘制半圆仪表盘
 * @param current_temp 当前温度
 * @param target_temp 目标温度
 */
void draw_semi_gauge(float current_temp, float target_temp)
{
    // 计算最大温度值（目标温度+10，向上取整到10的倍数）
    float max_temp = (float)round_up_to_10(target_temp + 10);

    // 屏幕左上区域绘制，圆心在左侧中间偏上
    int16_t center_x = 31;
    int16_t center_y = 30;
    int16_t radius = 30;

    // 1. 绘制半圆弧（-180°到0°，即左侧半圆）
    OLED_DrawArc(center_x, center_y, radius, -180, 0, OLED_UNFILLED);

    // 2. 绘制刻度点
    draw_semi_gauge_ticks(max_temp, center_x, center_y, radius);

    // 3. 绘制目标温度三角形
    draw_semi_target_marker(target_temp, max_temp, center_x, center_y, radius);

    // 4. 绘制温度指针
    draw_semi_temperature_pointer(current_temp, max_temp, center_x, center_y, 20);

    // 5. 在半圆下方显示WiFi IP地址（两行显示）
    const char* ip_addr = get_wifi_ip_address();

    if (strcmp(ip_addr, "No IP") != 0) {
        char ip_copy[16];
        strcpy(ip_copy, ip_addr);

        // 找到前两个点，分割IP地址
        char *first_dot = strchr(ip_copy, '.');
        char *second_dot = NULL;
        if (first_dot) {
            second_dot = strchr(first_dot + 1, '.');
        }

        if (first_dot && second_dot) {
            *second_dot = '\0';  // 截断前两段

            char first_part[20];
            char second_part[20];
            snprintf(first_part, sizeof(first_part), "%s.", ip_copy);  // 第一行带点: 192.168.
            snprintf(second_part, sizeof(second_part), "%s", second_dot + 1);  // 第二行带点: .50.223

            OLED_ShowString(8, 40, first_part, OLED_6X8);   // 第一行: 192.168.
            OLED_ShowString(8, 50, second_part, OLED_6X8);  // 第二行: .50.223
        }
    } else {
        char no_ip[] = "No IP";
        OLED_ShowString(0, 40, no_ip, OLED_6X8);
    }
}

// ========== PID初始化 ==========
void pid_init(pid_controller_t *pid, float target, float kp, float ki, float kd)
{
    pid->target_temp = target;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->last_error = 0.0f;
    pid->output = 0.0f;
    pid->pwm_duty = 0;
    ESP_LOGI(pidTAG, "PID初始化 - 目标:%.1f℃ Kp=%.0f Ki=%.0f Kd=%.0f", 
             target, kp, ki, kd);
}

// ========== PID计算 ==========
float pid_compute(pid_controller_t *pid, float current_temp, float dt)
{
    // 安全检查
    if (dt <= 0.001f || dt > 10.0f) {
        return pid->output;  // 保持上次输出
    }

    float error = pid->target_temp - current_temp;

    // 死区逻辑：如果误差在±1.0°C范围内，则不进行调整
    if (fabs(error) < 1.0f) {
        // 在死区内，逐渐降低输出，避免持续加热
        if (pid->output > 0) {
            pid->output *= 0.95f;  // 每个周期衰减5%
            if (pid->output < 10) pid->output = 0;  // 降到10以下直接归零
        }
        return pid->output;
    }

    // ========== P项 ==========
    float p_term = pid->kp * error;

    // ========== I项（移除限制） ==========
    pid->integral += error * dt;
    float i_term = pid->ki * pid->integral;

    // ========== D项 ==========
    float derivative = (error - pid->last_error) / dt;
    pid->filtered_derivative = derivative;  // 移除低通滤波
    float d_term = pid->kd * pid->filtered_derivative;

    pid->last_error = error;

    // ========== 输出计算 ==========
    pid->output = p_term + i_term + d_term;

    // 输出限幅
    if (pid->output > MAX_PWM_DUTY) pid->output = MAX_PWM_DUTY;
    else if (pid->output < MIN_PWM_DUTY) pid->output = MIN_PWM_DUTY;

    return pid->output;
}

// ========== 加热片PWM初始化 ==========
static void heater_ledc_init(void)
{
    ledc_timer_config_t heater_timer = {
        .speed_mode      = HEATER_LEDC_MODE,
        .duty_resolution = HEATER_DUTY_RES,
        .timer_num       = HEATER_LEDC_TIMER,
        .freq_hz         = HEATER_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&heater_timer));

    ledc_channel_config_t heater_channel = {
        .speed_mode = HEATER_LEDC_MODE,
        .channel    = HEATER_LEDC_CHANNEL,
        .timer_sel  = HEATER_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = HEATER_OUTPUT_IO,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&heater_channel));
    ESP_LOGI(pidTAG, "加热片PWM初始化完成 GPIO%d", HEATER_OUTPUT_IO);
}

// ========== 设置加热片PWM ==========
void set_heater_pwm(uint32_t duty)
{
    if (duty > MAX_PWM_DUTY) duty = MAX_PWM_DUTY;
    uint32_t scaled_duty = (duty * ((1 << HEATER_DUTY_RES) - 1)) / MAX_PWM_DUTY;  // 根据分辨率缩放占空比
    ledc_set_duty(HEATER_LEDC_MODE, HEATER_LEDC_CHANNEL, scaled_duty);
    ledc_update_duty(HEATER_LEDC_MODE, HEATER_LEDC_CHANNEL);
}

// ========== 风扇PWM初始化 ==========
static void fan_ledc_init(void)
{
    ledc_timer_config_t fan_timer = {
        .speed_mode      = FAN_LEDC_MODE,
        .duty_resolution = FAN_DUTY_RES,
        .timer_num       = FAN_LEDC_TIMER,
        .freq_hz         = FAN_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&fan_timer));

    ledc_channel_config_t fan_channel = {
        .speed_mode = FAN_LEDC_MODE,
        .channel    = FAN_LEDC_CHANNEL,
        .timer_sel  = FAN_LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = FAN_PWM_IO,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&fan_channel));
    ESP_LOGI(pidTAG, "风扇PWM初始化完成 GPIO%d", FAN_PWM_IO);
}

// ========== 设置风扇PWM（内部函数，不修改全局变量）==========
static void set_fan_pwm_internal(float percent)
{
    uint32_t duty;

    if (percent < 10.0f) percent = 0.0f;
    if (percent > 95.0f) percent = 95.0f;

    // 最小占空比限制：避免MOSFET在极低占空比时无法导通
    // 如果设置值 < 10%，强制为0（关闭风扇）
    // 如果设置值 >= 10%，最小设为20%以确保MOSFET可靠导通
    if (percent > 0.0f && percent < 10.0f) {
        percent = 0.0f;  // 低于10%直接关闭
    } else if (percent >= 10.0f && percent < 20.0f) {
        percent = 20.0f;  // 10-20%范围提升到20%
    }

    // 更新实际PWM输出百分比和运行状态
    FAN_ACTUAL_PWM = percent;
    FAN_IS_RUNNING = (percent > 0.0f);

    // 简化计算：直接将百分比映射到PWM占空比
    // 10位分辨率：0-1023，100% = 1023
    duty = (uint32_t)((percent * ((1 << FAN_DUTY_RES) - 1)) / 100.0f);

    ledc_set_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL, duty);
    ledc_update_duty(FAN_LEDC_MODE, FAN_LEDC_CHANNEL);

    // ESP_LOGI(pidTAG, "set_fan_pwm_internal: %.1f%% → duty=%u", percent, duty);
}

// ========== 设置风扇PWM（公开函数，修改全局变量）==========
void set_fan_pwm(float percent)
{
    // 先更新全局变量
    if (percent < 10.0f) percent = 0.0f;
    if (percent > 95.0f) percent = 95.0f;

    // 最小占空比限制逻辑
    if (percent > 0.0f && percent < 10.0f) {
        percent = 0.0f;
    } else if (percent >= 10.0f && percent < 20.0f) {
        percent = 20.0f;
    }

    FAN_SPEED_PERCENT = percent;

    // 调用内部函数设置硬件
    set_fan_pwm_internal(percent);

    // ESP_LOGI(pidTAG, "set_fan_pwm: FAN_SPEED_PERCENT 更新为 %.1f%%", FAN_SPEED_PERCENT);
}

// ========== 定时器回调函数 ==========
static void timer_callback(TimerHandle_t timer)
{
    ESP_LOGI(pidTAG, "⏰ 定时器到期，自动关闭系统");
    is_OPEN = false;
    system_start_time_ms = 0;  // 重置系统运行时间
    timer_is_running = false;  // 清除运行标志
    timer_start_time_ms = 0;   // 重置定时器启动时间
    // 注意：不删除 timer_handle，保留定时器以便下次使用
    // FreeRTOS 单次定时器到期后会自动停止，但句柄仍然有效
    ESP_LOGI(pidTAG, "✅ 定时器已自动关闭（句柄保留）");
}

// ========== 创建定时器 ==========
void create_system_timer(float hours)
{
    // 如果定时器已存在，先删除
    if (timer_handle != NULL) {
        xTimerStop(timer_handle, 0);
        xTimerDelete(timer_handle, 0);
        timer_handle = NULL;
    }

    // 如果定时器时长为0，不创建定时器
    if (hours <= 0.0f) {
        ESP_LOGI(pidTAG, "定时器未设置或为0，不创建定时器");
        return;
    }

    // 计算定时器时长（转换为tick）
    // 假设系统tick为1000Hz（1ms），需要将小时转换为毫秒
    uint32_t timer_duration_ms = (uint32_t)(hours * 3600.0f * 1000.0f);
    TickType_t timer_ticks = pdMS_TO_TICKS(timer_duration_ms);

    // 创建软件定时器（单次触发）
    timer_handle = xTimerCreate(
        "system_timer",         // 定时器名称
        timer_ticks,            // 定时器周期（ticks）
        pdFALSE,                // 单次触发（不自动重载）
        (void *)0,              // 定时器ID
        timer_callback          // 回调函数
    );

    if (timer_handle == NULL) {
        ESP_LOGE(pidTAG, "❌ 创建定时器失败");
    } else {
        ESP_LOGI(pidTAG, "✅ 定时器创建成功: %.1f小时 (%u毫秒)", hours, timer_duration_ms);
    }
}

// ========== 启动定时器 ==========
void start_system_timer(void)
{
    if (timer_handle != NULL) {
        ESP_LOGI(pidTAG, "🔍 尝试启动定时器: TIMER_HOURS_CONFIG=%.2f小时", TIMER_HOURS_CONFIG);
        if (xTimerStart(timer_handle, 0) == pdPASS) {
            timer_start_time_ms = esp_timer_get_time() / 1000;  // 记录启动时间（毫秒）
            timer_is_running = true;
            ESP_LOGI(pidTAG, "⏱️ 定时器已启动: %.2f小时, 启动时间=%lld", TIMER_HOURS_CONFIG, timer_start_time_ms);
        } else {
            ESP_LOGE(pidTAG, "❌ 启动定时器失败");
        }
    } else {
        ESP_LOGW(pidTAG, "⚠️ 定时器未创建，无法启动");
    }
}

// ========== 停止定时器（不删除） ==========
void stop_system_timer(void)
{
    if (timer_handle != NULL) {
        xTimerStop(timer_handle, 0);
        timer_is_running = false;
        timer_start_time_ms = 0;
        ESP_LOGI(pidTAG, "⏹️ 定时器已停止");
    }
}

// ========== 添加温度到曲线缓冲区 ==========
void add_temp_to_curve(float temp)
{
    temp_curve[curve_index] = temp;
    curve_index = (curve_index + 1) % CURVE_POINTS;
}

// ========== 绘制静态框架（只在初始化时调用一次）==========
void draw_static_frame(void)
{
    // 绘制边框
    for (uint8_t x = CURVE_START_X; x < CURVE_START_X+CURVE_WIDTH; x++) {
        OLED_DrawPoint(x, 0);
        OLED_DrawPoint(x, 63);
    }
    for (uint8_t y = 0; y < 64; y++) {
        OLED_DrawPoint(CURVE_START_X, y);
    }
    for (uint8_t y = 0; y < 64; y++) {
        OLED_DrawPoint(CURVE_START_X + CURVE_WIDTH+1, y);
    }
   
    // 绘制目标温度虚线
    uint8_t target_y = (uint8_t)(63 - ((TARGET_TEMP - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 62));
    for (uint8_t x = CURVE_START_X + 1; x < CURVE_START_X+CURVE_WIDTH; x++) {
        if ((x - CURVE_START_X) % 4 < 2) {
            OLED_DrawPoint(x, target_y);
        }
    }
}

// ========== 清除曲线区域（不清除边框和虚线）==========
void clear_curve_area(void)
{
    // 只清除曲线区域（不清除边框）
    for (uint8_t x = CURVE_START_X + 1; x < CURVE_START_X+CURVE_WIDTH; x++) {
        for (uint8_t y = 1; y < 63; y++) {
            // 跳过目标温度虚线的Y坐标
            uint8_t target_y = (uint8_t)(63 - ((TARGET_TEMP - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 62));
            if (y == target_y && (x - CURVE_START_X) % 4 < 2) {
                continue;  // 保留虚线
            }
            // 清除其他点（这里需要OLED库支持ClearPoint功能）
            // 由于你的OLED库只有DrawPoint，所以需要用ClearArea
        }
    }
}

// ========== 只绘制温度曲线（不画边框）==========
void draw_temp_curve_only(void)
{
    for (uint8_t i = 1; i < CURVE_POINTS; i++) {
        uint8_t idx2 = (curve_index + i) % CURVE_POINTS;
        float temp2 = temp_curve[idx2];
        
        if (temp2 < TEMP_MIN) temp2 = TEMP_MIN;
        if (temp2 > TEMP_MAX) temp2 = TEMP_MAX;
        
        uint8_t y2 = (uint8_t)(63 - ((temp2 - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 62));
        uint8_t x = CURVE_START_X + i;
        
        if (x < 128) {
            OLED_DrawPoint(x, y2);
        }
    }
}


// ========== 绘制右侧文字信息区域 ==========
/**
 * @brief 绘制右侧文字信息（温度、湿度、PID参数等）
 * @note 两种显示模式共用此函数
 */
void draw_right_text_area(void)
{
    char buffer[32];

    // 目标温度
    snprintf(buffer, sizeof(buffer), "SET:%.1fC", heater_pid.target_temp);
    OLED_ShowString(TEXT_START_X, 1, buffer, OLED_6X8);

    // PID参数
    snprintf(buffer, sizeof(buffer), "%.1f %.1f %.1f", PID_KP, PID_KI, PID_KD);
    OLED_ShowString(TEXT_START_X, 11, buffer, OLED_6X8);

    // NTC温度
    snprintf(buffer, sizeof(buffer), "NTC:%.1fC", ntcTemp);
    OLED_ShowString(TEXT_START_X, 21, buffer, OLED_6X8);

    // DHT22温度
    snprintf(buffer, sizeof(buffer), "DHT:%.1fC", dhtTemp);
    OLED_ShowString(TEXT_START_X, 31, buffer, OLED_6X8);

    // 湿度
    snprintf(buffer, sizeof(buffer), "HUM:%.1f%%", dhtHumidity);
    OLED_ShowString(TEXT_START_X, 41, buffer, OLED_6X8);

    // PWM输出百分比
    float pwm_percent = (heater_pid.pwm_duty * 100.0f) / MAX_PWM_DUTY;
    snprintf(buffer, sizeof(buffer), "PWM:%.1f%%", pwm_percent);
    OLED_ShowString(TEXT_START_X, 51, buffer, OLED_6X8);
}

// ========== 统一OLED显示任务 ==========
void oled_display_task(void *pvParameter)
{
    bool first_run = true;

    while(1) {
        // ========== 检查是否处于配网模式 ==========
        if (wifi_provisioning_is_active()) {
            // 配网模式下,不更新显示(由wifi_provisioning.c控制OLED)
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        xSemaphoreTake(oled_mutex, portMAX_DELAY);

#if DISPLAY_MODE == 0
        // ========== 模式0：曲线+文字 ==========

        // 只清除右侧文字区域
        OLED_ClearArea(TEXT_START_X, 0, 128, 64);

        // 绘制右侧文字信息
        draw_right_text_area();

        // 左侧温度曲线区
        if (first_run) {
            draw_static_frame();
            first_run = false;
        }

        OLED_ClearArea(CURVE_START_X + 1, 1, 62, 62);

        uint8_t target_y = (uint8_t)(63 - ((TARGET_TEMP - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 62));
        for (uint8_t x = CURVE_START_X + 1; x < CURVE_START_X+CURVE_WIDTH; x++) {
            if ((x - CURVE_START_X) % 4 < 2) {
                OLED_DrawPoint(x, target_y);
            }
        }

        draw_temp_curve_only();

#elif DISPLAY_MODE == 1
        // ========== 模式1：仪表盘+文字 ==========

        // 清除整个屏幕（仪表盘需要重绘）
        OLED_Clear();

        // 左侧绘制仪表盘
        draw_gauge(ntcTemp, heater_pid.target_temp);

        // 绘制右侧文字信息
        draw_right_text_area();

#elif DISPLAY_MODE == 2
        // ========== 模式2：半圆仪表盘+文字 ==========

        // 清除整个屏幕
        OLED_Clear();

        // 左侧绘制半圆仪表盘
        draw_semi_gauge(ntcTemp, heater_pid.target_temp);

        // 绘制右侧文字信息
        draw_right_text_area();

#endif

        OLED_Update();
        xSemaphoreGive(oled_mutex);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// ========== PID温度控制任务 ==========
void pid_temperature_control_task(void *pvParameter)
{
    float dt = PID_INTERVAL_MS / 1000.0f;
    ESP_LOGI(pidTAG, "PID温控任务启动");
    
    while(1) {
        float current_temp = get_temp();
        ntcTemp = current_temp;
        
        // 添加温度到曲线（模式0使用）
        add_temp_to_curve(current_temp);
        
        if (is_OPEN) {
            // 超温保护：温度超过目标值5度时强制关闭
            if (current_temp > heater_pid.target_temp + 5.0f) {
                ESP_LOGW(pidTAG, "⚠️ 超温保护！%.2f℃ > 目标+5℃", current_temp);
                set_heater_pwm(0);
                heater_pid.pwm_duty = 0;
                heater_pid.integral = 0;  // 清零积分项
                set_fan_pwm(100.0f);  // 超温时风扇全速
                vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
                continue;
            }

            // 极限温度保护
            if (current_temp > MAX_TEMP_LIMIT) {
                ESP_LOGE(pidTAG, "🔥 极限温度保护！%.2f℃", current_temp);
                set_heater_pwm(0);
                heater_pid.pwm_duty = 0;
                heater_pid.integral = 0;
                set_fan_pwm(100.0f);  // 风扇全速
                is_OPEN = false;  // 自动关闭系统
                vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
                continue;
            }

            // PID计算
            float pid_output = pid_compute(&heater_pid, current_temp, dt);
            heater_pid.pwm_duty = (uint32_t)pid_output;
            set_heater_pwm(heater_pid.pwm_duty);

            // 启动风扇（使用PWM控制转速）
            set_fan_pwm(FAN_SPEED_PERCENT);
        } else {
            // 如果 is_OPEN 为 false，关闭加热片
            heater_pid.pwm_duty=0;
            set_heater_pwm(heater_pid.pwm_duty);

            // 检查温度是否低于 35°C
            if (current_temp < 35.0f) {
                set_fan_pwm_internal(0.0f);  // 使用内部函数，不修改FAN_SPEED_PERCENT全局变量
                static uint32_t log_count = 0;
                if (++log_count % 25 == 0) {  // 每5秒打印一次
                    ESP_LOGI(pidTAG, "系统关闭, 温度%.1f°C<35°C, 风扇关闭", current_temp);
                }
            } else {
                set_fan_pwm_internal(FAN_SPEED_PERCENT);  // 使用配置的风扇转速（不修改全局变量）
                static uint32_t log_count2 = 0;
                if (++log_count2 % 25 == 0) {  // 每5秒打印一次
                    ESP_LOGI(pidTAG, "系统关闭, 温度%.1f°C>=35°C, 风扇%.1f%%", current_temp, FAN_SPEED_PERCENT);
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}

// ========== DHT22任务 ==========
void tesk_dht22(void *pvParameter)
{
    while(1) {
        xSemaphoreTake(dht22_mutex, portMAX_DELAY);
        int ret = readDHT();
        if (ret == DHT_OK) {
            dhtTemp = getTemperature();
            dhtHumidity = getHumidity();
            if (dhtFailCount > 0) {
                ESP_LOGI(dhtTAG, "DHT22恢复在线，温度:%.1f℃ 湿度:%.1f%%", dhtTemp, dhtHumidity);
            }
            dhtFailCount = 0;
            dhtAvailable = true;
            dhtHasValidReading = true;
        } else {
            dhtFailCount++;
            if (dhtFailCount >= 3) {
                dhtAvailable = false;
            }
            if (dhtFailCount == 1 || dhtFailCount % 15 == 0) {
                ESP_LOGW(dhtTAG, "DHT22读取失败: %d, 连续失败:%" PRIu32, ret, dhtFailCount);
            }
        }
        xSemaphoreGive(dht22_mutex);
        
        // ESP_LOGI(dhtTAG, "温度:%.1f℃ 湿度:%.1f%%", dhtTemp, dhtHumidity); //SEE
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ========== 呼吸灯初始化 ==========
static void LEDbubble_ledc_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = LEDC_OUTPUT_IO,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// ========== 呼吸灯任务 ==========
void change_duty(void *pvParameter)
{
    while(1) {
        for (int i = 0; i < 8192; i += 220) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
        for (int j = 8192; j > 0; j -= 220) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, j);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
}

// ========== 主函数 ==========
void app_main(void)
{
    //NVS初始化（WIFI底层驱动有用到NVS，所以这里要初始化）
    nvs_flash_init();

    // 检查 NVS 中存储的风扇转速
    nvs_handle_t test_handle;
    if (nvs_open("storage", NVS_READONLY, &test_handle) == ESP_OK) {
        float test_fan_speed;
        size_t size = sizeof(test_fan_speed);
        esp_err_t err = nvs_get_blob(test_handle, "fan_speed", &test_fan_speed, &size);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "🔍 [NVS检查] NVS 中存储的风扇转速: %.1f%%", test_fan_speed);
        } else {
            ESP_LOGW(TAG, "🔍 [NVS检查] NVS 中没有风扇转速数据");
        }
        nvs_close(test_handle);
    } else {
        ESP_LOGE(TAG, "🔍 [NVS检查] 无法打开 NVS");
    }

    // ========== 先初始化OLED(配网模式需要用到) ==========
    // 创建信号量
    dht22_mutex = xSemaphoreCreateMutex();
    oled_mutex = xSemaphoreCreateMutex();

    // 初始化OLED
    ESP_LOGI(screenTAG, "初始化OLED...");
    OLED_Init(OLED_I2C, OLED_ADD, OLED_SCL, OLED_SDA, OLED_SPEED);
    OLED_Clear();

    // 初始化INA226，和OLED共用同一条I2C总线
    esp_err_t ina226_err = ina226_monitor_init();
    if (ina226_err != ESP_OK) {
        ESP_LOGW(TAG, "INA226初始化失败: %s", esp_err_to_name(ina226_err));
    }

    // 初始化呼吸灯
    LEDbubble_ledc_init();

    // 初始化风扇PWM硬件(不设置转速,由wifi_sta_init中从NVS加载)
    fan_ledc_init();

    // 初始化加热片
    heater_ledc_init();

    // 初始化PID
    pid_init(&heater_pid, TARGET_TEMP, PID_KP, PID_KI, PID_KD);

    // 初始化NTC
    temp_ntc_init();

    // 初始化DHT22
    setDHTgpio(GPIO_NUM_0);

    // ========== 后初始化WiFi(可能进入配网模式,需要OLED已初始化) ==========
    //wifi STA工作模式初始化
    wifi_sta_init();

    // 创建任务
    xTaskCreatePinnedToCore(change_duty, "led_breath", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(tesk_dht22, "dht22", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(pid_temperature_control_task, "pid_ctrl", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(oled_display_task, "oled_disp", 3072, NULL, 3, NULL, 0);
    if (ina226_err == ESP_OK) {
        xTaskCreatePinnedToCore(ina226_monitor_task, "ina226", 3072, NULL, 3, NULL, 0);
    }

    ESP_LOGI(TAG, "🚀 系统启动完成！");
    ESP_LOGI(TAG, "风扇配置: 转速=%.1f%%, GPIO=%d", FAN_SPEED_PERCENT, FAN_PWM_IO);
    ESP_LOGI(TAG, "当前状态: is_OPEN=%s", is_OPEN ? "开启" : "关闭");


}
