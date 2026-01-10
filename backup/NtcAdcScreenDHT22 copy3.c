/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * 【√】显示adc、dht
 * 【√】增加pid控温
 * 【√】温度曲线
 * 【】阶段优化代码
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

// ========== OLED配置 ==========
#define OLED_I2C    I2C_NUM_0
#define OLED_SCL    GPIO_NUM_4
#define OLED_SDA    GPIO_NUM_5
#define OLED_ADD    0x78
#define OLED_SPEED  400000

// ========== 温度曲线配置 ==========
#define CURVE_START_X   70      // 曲线起始X坐标（右半屏）
#define CURVE_WIDTH     58      // 曲线宽度
#define CURVE_HEIGHT    64      // 曲线高度
#define CURVE_POINTS    58      // 曲线数据点数量
#define TEMP_MIN        20.0f   // 曲线显示最低温度
#define TEMP_MAX        40.0f   // 曲线显示最高温度

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
#define TARGET_TEMP      30.0f
#define PID_KP           150.0f
#define PID_KI           5.0f
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
    float output;
    uint32_t pwm_duty;
} pid_controller_t;

// ========== 全局变量 ==========
pid_controller_t heater_pid;
float ntcTemp = 0.0f;
float dhtTemp = 0.0f;
float dhtHumidity = 0.0f;
static uint8_t last_curve_y[CURVE_POINTS] = {0};
static bool first_draw = true;
static bool static_frame_drawn = false;

// ========== 温度曲线缓冲区 ==========
static float temp_curve[CURVE_POINTS] = {0};
static uint8_t curve_index = 0;

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
    float error = pid->target_temp - current_temp;
    float p_term = pid->kp * error;
    
    pid->integral += error * dt;
    float max_integral = MAX_PWM_DUTY / pid->ki;
    if (pid->integral > max_integral) pid->integral = max_integral;
    else if (pid->integral < -max_integral) pid->integral = -max_integral;
    float i_term = pid->ki * pid->integral;
    
    float derivative = (error - pid->last_error) / dt;
    float d_term = pid->kd * derivative;
    pid->last_error = error;
    
    pid->output = p_term + i_term + d_term;
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
    for (uint8_t x = CURVE_START_X; x < 128; x++) {
        OLED_DrawPoint(x, 0);
        OLED_DrawPoint(x, 63);
    }
    for (uint8_t y = 0; y < 64; y++) {
        OLED_DrawPoint(CURVE_START_X, y);
    }
    
    // 绘制目标温度虚线
    uint8_t target_y = (uint8_t)(63 - ((TARGET_TEMP - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 62));
    for (uint8_t x = CURVE_START_X + 1; x < 128; x++) {
        if ((x - CURVE_START_X) % 4 < 2) {
            OLED_DrawPoint(x, target_y);
        }
    }
}

// ========== 清除曲线区域（不清除边框和虚线）==========
void clear_curve_area(void)
{
    // 只清除曲线区域（不清除边框）
    for (uint8_t x = CURVE_START_X + 1; x < 128; x++) {
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
        
        // ========== 只清除左侧文字区域 ==========
        OLED_ClearArea(0, 0, 64, 64);  // 只清除左半屏
        
        // ========== 左侧文字区域 ==========
        snprintf(buffer, sizeof(buffer), "Set:%.0fC", heater_pid.target_temp);
        OLED_ShowString(0, 0, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "P%.0f I%.0f D%.0f", PID_KP, PID_KI, PID_KD);
        OLED_ShowString(0, 10, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "NTC:%.1fC", ntcTemp);
        OLED_ShowString(0, 20, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "DHT:%.1fC", dhtTemp);
        OLED_ShowString(0, 30, buffer, OLED_6X8);
        
        snprintf(buffer, sizeof(buffer), "H:%.0f%%", dhtHumidity);
        OLED_ShowString(0, 40, buffer, OLED_6X8);
        
        float pwm_percent = (heater_pid.pwm_duty * 100.0f) / MAX_PWM_DUTY;
        snprintf(buffer, sizeof(buffer), "PWM:%.0f%%", pwm_percent);
        OLED_ShowString(0, 50, buffer, OLED_6X8);
        
        // ========== 右侧温度曲线区 ==========
        if (first_run) {
            // 第一次绘制边框和虚线
            draw_static_frame();
            first_run = false;
        }
        
        // 每次清除曲线区域并重绘
        OLED_ClearArea(CURVE_START_X + 1, 1, 63, 62);  // 清除曲线区域（不清除边框）
        
        // 重绘虚线（因为被清除了）
        uint8_t target_y = (uint8_t)(63 - ((TARGET_TEMP - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 62));
        for (uint8_t x = CURVE_START_X + 1; x < 128; x++) {
            if ((x - CURVE_START_X) % 4 < 2) {
                OLED_DrawPoint(x, target_y);
            }
        }
        
        // 绘制温度曲线
        draw_temp_curve_only();
        
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
        
        // 添加温度到曲线
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
