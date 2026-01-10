/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * 【√】显示adc、dht
 * 【√】增加pid控温
 * 【√】温度曲线，挪到左边
 * 【√】温度码表，优化pid
 * 【】阶段优化冗余代码
 * 【】通风换气
 * 【】联网设置温度
 * 【】联网mqtt显示数据
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <string.h>
#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "dht22.h"
#include "driver/ledc.h"
#include "ntc.h"
#include "esp_err.h"
#include "OLED.h"

#define TAG       "main"
#define ntcTAG    "ntc"
#define dhtTAG    "dht22"
#define screenTAG "screen"
#define ledTAG    "led"
#define pidTAG    "PID"

// ========== 显示模式选择 ==========
#define DISPLAY_MODE    1    // 0=曲线+文字  1=仪表盘+文字

// ========== OLED配置 ==========
#define OLED_I2C    I2C_NUM_0
#define OLED_SCL    GPIO_NUM_4
#define OLED_SDA    GPIO_NUM_5
#define OLED_ADD    0x78
#define OLED_SPEED  400000

// ========== 温度曲线配置 ==========
#define CURVE_START_X   0      // 曲线起始X坐标（右半屏）
#define CURVE_WIDTH     60      // 曲线宽度
#define CURVE_HEIGHT    64      // 曲线高度
#define CURVE_POINTS    56      // 曲线数据点数量
#define TEXT_START_X    64      //文字开始位置
#define TEMP_MIN        20.0f   // 曲线显示最低温度
#define TEMP_MAX        80.0f   // 曲线显示最高温度

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

// ========== 信号量 ==========
SemaphoreHandle_t dht22_mutex;
SemaphoreHandle_t oled_mutex;

// ========== 呼吸灯配置 (GPIO 8) ==========
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO  (8)
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_DUTY_RES   LEDC_TIMER_13_BIT
#define LEDC_FREQUENCY  (1000)

// ========== 加热片PWM配置 (GPIO 10) ==========
#define HEATER_LEDC_TIMER    LEDC_TIMER_1
#define HEATER_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define HEATER_OUTPUT_IO     (10)
#define HEATER_LEDC_CHANNEL  LEDC_CHANNEL_1
#define HEATER_DUTY_RES      LEDC_TIMER_13_BIT
#define HEATER_FREQUENCY     (1000)

// ========== PID控制参数 ==========
#define TARGET_TEMP      33.0f    //目标温度
#define PID_KP           100.0f   
#define PID_KI           2.0f
#define PID_KD           50.0f
#define PID_INTERVAL_MS  500
#define MAX_TEMP_LIMIT   100.0f
#define MIN_TEMP_LIMIT   0.0f
#define MAX_PWM_DUTY     6500
#define MIN_PWM_DUTY     0

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

// ========== 数学辅助宏 ==========
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


// ========== 温度曲线缓冲区 ==========
static float temp_curve[CURVE_POINTS] = {0};
static uint8_t curve_index = 0;

// ========== 仪表盘绘制函数 ==========

/**
 * @brief 将温度转换为角度
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
 * @brief 绘制目标温度三角形标记
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
 * @brief 绘制温度指针
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
 * @brief 绘制完整仪表盘
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
    
    // ========== P项 ==========
    float p_term = pid->kp * error;
    
    // ========== I项（带积分分离） ==========
    // 误差大于5°C时禁用积分，防止超调
    if (fabs(error) < 5.0f) {
        pid->integral += error * dt;
        
        // 积分限幅（抗饱和）
        float max_integral = MAX_PWM_DUTY / (pid->ki + 0.0001f);  // 防止除零
        if (pid->integral > max_integral) pid->integral = max_integral;
        else if (pid->integral < 0) pid->integral = 0;  // 温控不需要负积分
    }
    float i_term = pid->ki * pid->integral;
    
    // ========== D项（带低通滤波） ==========
    float derivative = (error - pid->last_error) / dt;
    
    // 一阶低通滤波，平滑微分项（截止频率约1.6Hz）
    pid->filtered_derivative = 0.9f * pid->filtered_derivative + 0.1f * derivative;
    float d_term = pid->kd * pid->filtered_derivative;
    
    pid->last_error = error;
    
    // ========== 输出计算 ==========
    pid->output = p_term + i_term + d_term;
    
    // 死区控制：±0.5°C内不调整（减少继电器动作）
    if (fabs(error) < 0.5f) {
        // 保持当前输出或缓慢衰减
        pid->output *= 0.95f;
    }
    
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
    ledc_set_duty(HEATER_LEDC_MODE, HEATER_LEDC_CHANNEL, duty);
    ledc_update_duty(HEATER_LEDC_MODE, HEATER_LEDC_CHANNEL);
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


// ========== 统一OLED显示任务 ==========
void oled_display_task(void *pvParameter)
{
    char buffer[32];
    bool first_run = true;
    
    while(1) {
        xSemaphoreTake(oled_mutex, portMAX_DELAY);
        
#if DISPLAY_MODE == 0
        // ========== 模式0：曲线+文字 ==========
        
        // 只清除右侧文字区域
        OLED_ClearArea(TEXT_START_X, 0, 128, 64);
        
        // 右侧文字区域
        snprintf(buffer, sizeof(buffer), "SET:%.1fC", heater_pid.target_temp);
        OLED_ShowString(TEXT_START_X, 0, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "%.1f %.1f %.1f", PID_KP, PID_KI, PID_KD);
        OLED_ShowString(TEXT_START_X, 10, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "NTC:%.1fC", ntcTemp);
        OLED_ShowString(TEXT_START_X, 23, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "DHT:%.1fC", dhtTemp);
        OLED_ShowString(TEXT_START_X, 33, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "HUM:%.1f%%", dhtHumidity);
        OLED_ShowString(TEXT_START_X, 43, buffer, OLED_6X8);
        
        float pwm_percent = (heater_pid.pwm_duty * 100.0f) / MAX_PWM_DUTY;
        snprintf(buffer, sizeof(buffer), "PWM:%.1f%%", pwm_percent);
        OLED_ShowString(TEXT_START_X, 56, buffer, OLED_6X8);
        
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
        
        // 右侧文字区域（与模式0相同）
        snprintf(buffer, sizeof(buffer), "SET:%.1fC", heater_pid.target_temp);
        OLED_ShowString(TEXT_START_X, 0, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "%.1f %.1f %.1f", PID_KP, PID_KI, PID_KD);
        OLED_ShowString(TEXT_START_X, 10, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "NTC:%.1fC", ntcTemp);
        OLED_ShowString(TEXT_START_X, 23, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "DHT:%.1fC", dhtTemp);
        OLED_ShowString(TEXT_START_X, 33, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "HUM:%.1f%%", dhtHumidity);
        OLED_ShowString(TEXT_START_X, 43, buffer, OLED_6X8);
        
        float pwm_percent = (heater_pid.pwm_duty * 100.0f) / MAX_PWM_DUTY;
        snprintf(buffer, sizeof(buffer), "PWM:%.1f%%", pwm_percent);
        OLED_ShowString(TEXT_START_X, 56, buffer, OLED_6X8);
        
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
        
        // 超温保护
        if (current_temp > MAX_TEMP_LIMIT) {
            ESP_LOGW(pidTAG, "⚠️ 超温保护！%.2f℃", current_temp);
            set_heater_pwm(0);
            heater_pid.integral = 0;
            vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
            continue;
        }
        
        // PID计算
        float pid_output = pid_compute(&heater_pid, current_temp, dt);
        heater_pid.pwm_duty = (uint32_t)pid_output;
        set_heater_pwm(heater_pid.pwm_duty);
        
        float pwm_percent = (heater_pid.pwm_duty * 100.0f) / MAX_PWM_DUTY;
        ESP_LOGI(pidTAG, "目标:%.1f | NTC:%.2f | 误差:%.2f | PWM:%.0f%%", 
                 heater_pid.target_temp, current_temp, 
                 heater_pid.target_temp - current_temp, pwm_percent);
        
        vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}

// ========== DHT22任务 ==========
void tesk_dht22(void *pvParameter)
{
    while(1) {
        xSemaphoreTake(dht22_mutex, portMAX_DELAY);
        int ret = readDHT();
        errorHandler(ret);
        dhtTemp = getTemperature();
        dhtHumidity = getHumidity();
        xSemaphoreGive(dht22_mutex);
        
        ESP_LOGI(dhtTAG, "温度:%.1f℃ 湿度:%.1f%%", dhtTemp, dhtHumidity);
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
        for (int i = 0; i < 8192; i += 80) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
        for (int j = 8192; j > 0; j -= 80) {
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, j);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    }
}

// ========== 主函数 ==========
void app_main(void)
{
    // 初始化呼吸灯
    LEDbubble_ledc_init();
    
    // 初始化加热片
    heater_ledc_init();
    
    // 初始化PID
    pid_init(&heater_pid, TARGET_TEMP, PID_KP, PID_KI, PID_KD);
    
    // 初始化NTC
    temp_ntc_init();
    
    // 初始化DHT22
    setDHTgpio(GPIO_NUM_9);
    
    // 创建信号量
    dht22_mutex = xSemaphoreCreateMutex();
    oled_mutex = xSemaphoreCreateMutex();
    
    // 初始化OLED
    ESP_LOGI(screenTAG, "初始化OLED...");
    OLED_Init(OLED_I2C, OLED_ADD, OLED_SCL, OLED_SDA, OLED_SPEED);
    OLED_Clear();
    
    // 创建任务
    xTaskCreatePinnedToCore(change_duty, "led_breath", 2048, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(tesk_dht22, "dht22", 2048, NULL, 3, NULL, 0);
    xTaskCreatePinnedToCore(pid_temperature_control_task, "pid_ctrl", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(oled_display_task, "oled_disp", 3072, NULL, 3, NULL, 0);
    
    ESP_LOGI(TAG, "🚀 系统启动完成！");
}
