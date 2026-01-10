/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * 【√】显示adc、dht
 * 【√】增加pid控温
 * 【】温度曲线
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


#define TAG     "main"
#define ntcTAG  "ntc"
#define dhtTAG  "dht22"
#define screenTAG "screen"
#define ledTAG   "led"
#define pidTAG   "PID"

#define OLED_I2C    I2C_NUM_0
#define OLED_SCL    GPIO_NUM_4
#define OLED_SDA    GPIO_NUM_5
#define OLED_ADD    0x78    //0x78
#define OLED_SPEED  400000
#define DELAY_TIME 3000

SemaphoreHandle_t dht22_mutex;
SemaphoreHandle_t oled_mutex;
SemaphoreHandle_t ntc_mutex;


// =========呼吸灯 gpio8 ==============
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (8) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits 2^13=8192
#define LEDC_DUTY               (4096) // Set duty to 50%. (2 ** 13) * 50% = 4096
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 4 kHz  pwm频率

// ========== 加热片PWM配置 (GPIO 10) ==========
#define HEATER_LEDC_TIMER       LEDC_TIMER_1
#define HEATER_LEDC_MODE        LEDC_LOW_SPEED_MODE
#define HEATER_OUTPUT_IO        (10) // 加热片控制引脚
#define HEATER_LEDC_CHANNEL     LEDC_CHANNEL_1
#define HEATER_DUTY_RES         LEDC_TIMER_13_BIT // 13位分辨率 0-8191
#define HEATER_FREQUENCY        (1000) // 1kHz PWM频率

// ========== PID控制参数 ==========
#define TARGET_TEMP             30.0f    // 目标温度 30℃
#define PID_KP                  150.0f   // 比例系数
#define PID_KI                  5.0f     // 积分系数
#define PID_KD                  50.0f    // 微分系数
#define PID_INTERVAL_MS         500      // PID控制周期 500ms
#define MAX_TEMP_LIMIT          100.0f   // 最高温度限制 设置为100摄氏度
#define MIN_TEMP_LIMIT          0.0f     // 最低温度限制
#define MAX_PWM_DUTY            6500     // 最大PWM占空比 80%左右
#define MIN_PWM_DUTY            0        // 最小PWM占空比

// ========== PID结构体 ==========
typedef struct {
    float target_temp;      // 目标温度
    float kp;               // 比例系数
    float ki;               // 积分系数
    float kd;               // 微分系数
    float integral;         // 积分累积
    float last_error;       // 上次误差
    float output;           // PID输出值
    uint32_t pwm_duty;      // PWM占空比
} pid_controller_t;

float ntcTemp;
pid_controller_t heater_pid;

// ========== PID控制器初始化 ==========
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
    ESP_LOGI(pidTAG, "PID初始化完成 - 目标温度: %.1f℃, Kp=%.1f, Ki=%.1f, Kd=%.1f", 
             target, kp, ki, kd);
}

// ========== PID计算函数 ==========
float pid_compute(pid_controller_t *pid, float current_temp, float dt)
{
    // 计算误差
    float error = pid->target_temp - current_temp;
    
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项（带抗积分饱和）
    pid->integral += error * dt;
    // 积分限幅，防止积分饱和
    float max_integral = MAX_PWM_DUTY / pid->ki;
    if (pid->integral > max_integral) {
        pid->integral = max_integral;
    } else if (pid->integral < -max_integral) {
        pid->integral = -max_integral;
    }
    float i_term = pid->ki * pid->integral;
    
    // 微分项
    float derivative = (error - pid->last_error) / dt;
    float d_term = pid->kd * derivative;
    
    // 保存当前误差
    pid->last_error = error;
    
    // PID输出
    pid->output = p_term + i_term + d_term;
    
    // 输出限幅
    if (pid->output > MAX_PWM_DUTY) {
        pid->output = MAX_PWM_DUTY;
    } else if (pid->output < MIN_PWM_DUTY) {
        pid->output = MIN_PWM_DUTY;
    }
    
    return pid->output;
}

// ========== 加热片PWM初始化 ==========
static void heater_ledc_init(void)
{
    // 配置定时器
    ledc_timer_config_t heater_timer = {
        .speed_mode       = HEATER_LEDC_MODE,
        .duty_resolution  = HEATER_DUTY_RES,
        .timer_num        = HEATER_LEDC_TIMER,
        .freq_hz          = HEATER_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&heater_timer));

    // 配置通道
    ledc_channel_config_t heater_channel = {
        .speed_mode     = HEATER_LEDC_MODE,
        .channel        = HEATER_LEDC_CHANNEL,
        .timer_sel      = HEATER_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = HEATER_OUTPUT_IO,
        .duty           = 0, // 初始占空比为0
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&heater_channel));
    
    ESP_LOGI(pidTAG, "加热片PWM初始化完成 - GPIO%d, 频率%dHz", HEATER_OUTPUT_IO, HEATER_FREQUENCY);
}

// ========== 设置加热片PWM占空比 ==========
void set_heater_pwm(uint32_t duty)
{
    if (duty > MAX_PWM_DUTY) {
        duty = MAX_PWM_DUTY;
    }
    ledc_set_duty(HEATER_LEDC_MODE, HEATER_LEDC_CHANNEL, duty);
    ledc_update_duty(HEATER_LEDC_MODE, HEATER_LEDC_CHANNEL);
}

// ========== PID温度控制任务 ==========
void pid_temperature_control_task(void *pvParameter)
{
    float dt = PID_INTERVAL_MS / 1000.0f; // 转换为秒
    
    ESP_LOGI(pidTAG, "PID温度控制任务启动");
    
    while(1) {
        // 获取当前NTC温度
        float current_temp = get_temp();
        
        // 安全检查：超温保护
        if (current_temp > MAX_TEMP_LIMIT) {
            ESP_LOGW(pidTAG, "⚠️ 温度过高！当前: %.2f℃ > 限制: %.2f℃，强制关闭加热", 
                     current_temp, MAX_TEMP_LIMIT);
            set_heater_pwm(0);
            heater_pid.integral = 0; // 清除积分项
            vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
            continue;
        }
        
        // 计算PID输出
        float pid_output = pid_compute(&heater_pid, current_temp, dt);
        heater_pid.pwm_duty = (uint32_t)pid_output;
        
        // 设置PWM占空比
        set_heater_pwm(heater_pid.pwm_duty);
        
        // 计算PWM百分比
        float pwm_percent = (heater_pid.pwm_duty * 100.0f) / MAX_PWM_DUTY;
        
        // 日志输出
        ESP_LOGI(pidTAG, "目标: %.1f℃ | 当前: %.2f℃ | 误差: %.2f℃ | PWM: %lu (%.1f%%)", 
                 heater_pid.target_temp, 
                 current_temp, 
                 heater_pid.target_temp - current_temp,
                 heater_pid.pwm_duty,
                 pwm_percent);
        
        // 在OLED上显示PID信息
        xSemaphoreTake(oled_mutex, portMAX_DELAY);
        char buffer[32];
        
        // 显示目标温度
        snprintf(buffer, sizeof(buffer), "Set Temp:%.1f ", heater_pid.target_temp);
        OLED_ShowString(0, 0, buffer, OLED_6X8);
        // PID参数 
        snprintf(buffer, sizeof(buffer), "P%.1f I%.1f D%.1f  ", PID_KP,PID_KI,PID_KD);
        OLED_ShowString(0, 10,buffer, OLED_6X8);
        // 显示PWM百分比
        snprintf(buffer, sizeof(buffer), "PWM:%d%%  ", (int)pwm_percent);
        OLED_ShowString(0, 50, buffer, OLED_6X8);

        OLED_Update();
        xSemaphoreGive(oled_mutex);
        
        vTaskDelay(pdMS_TO_TICKS(PID_INTERVAL_MS));
    }
}


// ========== dht22 测试 =============
void tesk_dht22(void *pvParameter)
{
    // int temp,humidity ;
    float temp;
    float humidity;
    while(1){
        xSemaphoreTake(dht22_mutex, portMAX_DELAY);
        int ret = readDHT();
        vTaskDelay(pdMS_TO_TICKS(1000));
        errorHandler(ret);
        temp = getTemperature();
        humidity = getHumidity();
        // ESP_LOGI("dht11", "teskA temp: %d", temp);
        ESP_LOGI(dhtTAG, "temperature: %.1f℃", temp);
        ESP_LOGI(dhtTAG, "humidity   : %.1f/%\n", humidity);
        xSemaphoreGive(dht22_mutex);
        // 更新OLED显示SEE
        xSemaphoreTake(oled_mutex, portMAX_DELAY);

        char ntc_buffer[16];
        snprintf(ntc_buffer, sizeof(ntc_buffer), "DHT-T:%.1f  ", temp);
        OLED_ShowString(0, 30,ntc_buffer, OLED_6X8);
        snprintf(ntc_buffer, sizeof(ntc_buffer), "H:%.1f  ", humidity);
        OLED_ShowString(80, 30,ntc_buffer, OLED_6X8);
        OLED_Update();

        xSemaphoreGive(oled_mutex);  // ⭐ 关键：释放信号量！
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void LEDbubble_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

// 任务函数：改变LED的占空比
void change_duty(void *pvParameter)
{
    while(1){
    for (int i =0;i<8192;i+=80)
        {
            // Set duty to 50%
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, i);
            // Update duty to apply the new value
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    ESP_LOGI(ledTAG, "pwm=100");
    for (int j =8192;j>0;j-=80)
        {
            // Set duty to 50%
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL,j);
            // Update duty to apply the new value
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(25));
        }
    ESP_LOGI(ledTAG, "pwm=0");
    }
}

void NTC_TEST(void *pvParameter)
{
    while(1)
        {
            ntcTemp = get_temp();
            ESP_LOGI(ntcTAG,"temp:%.2f℃",ntcTemp);
            // ⭐ 添加互斥锁保护
            xSemaphoreTake(oled_mutex, portMAX_DELAY);
            char ntc_buffer[16];
            // 手动分离整数和小数部分
            int temp_int = (int)ntcTemp;
            int temp_dec = (int)((ntcTemp - temp_int) * 10);  // 保留1位小数
            snprintf(ntc_buffer, sizeof(ntc_buffer), "NTC-T:%d.%d  ",temp_int,temp_dec);
            OLED_ShowString(0, 20,ntc_buffer, OLED_6X8);

            OLED_Update();
            xSemaphoreGive(oled_mutex);  // ⭐ 释放互斥锁

            vTaskDelay(pdMS_TO_TICKS(1000));
        }
}

void app_main(void)
{
    // Set the LEDC peripheral configuration
    LEDbubble_ledc_init();
    
    // 初始化加热片PWM (GPIO 10)
    heater_ledc_init();

    // 初始化PID控制器
    pid_init(&heater_pid, TARGET_TEMP, PID_KP, PID_KI, PID_KD);

    // 初始化 ntc
    temp_ntc_init();

    // 初始化DHT22
    setDHTgpio(GPIO_NUM_9);

    // 创建信号量
    dht22_mutex = xSemaphoreCreateMutex();
    oled_mutex = xSemaphoreCreateMutex();

        // 初始化OLED
    ESP_LOGI(screenTAG, "Initializing OLED...");
    OLED_Init(OLED_I2C, OLED_ADD, OLED_SCL, OLED_SDA, OLED_SPEED);
    OLED_Clear();


    // rtos循环
    xTaskCreatePinnedToCore(change_duty, "change_duty", 2048, NULL, 3, NULL, 0);

    xTaskCreatePinnedToCore(tesk_dht22, "tesk_dht22", 2048, NULL, 3, NULL, 0);

    xTaskCreatePinnedToCore(NTC_TEST, "ntc_test", 2048, NULL, 3, NULL, 0);

    // ⭐ 新增：PID温度控制任务
    xTaskCreatePinnedToCore(pid_temperature_control_task, "pid_control", 4096, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "所有任务已创建，系统启动完成");
}
