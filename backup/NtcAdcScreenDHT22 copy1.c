/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 * 【√】显示adc、dht
 * 【】增加pid控温
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




float ntcTemp;

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
            ESP_LOGI(ntcTAG,"temp:%.2f℃",get_temp());
            vTaskDelay(pdMS_TO_TICKS(1000));
            char ntc_buffer[16];
            ntcTemp = get_temp();
            // 手动分离整数和小数部分
            int temp_int = (int)ntcTemp;
            int temp_dec = (int)((ntcTemp - temp_int) * 10);  // 保留1位小数
            snprintf(ntc_buffer, sizeof(ntc_buffer), "NTC-T:%d.%d  ",temp_int,temp_dec);
            OLED_ShowString(0, 20,ntc_buffer, OLED_6X8);
        }
}

void app_main(void)
{
    // Set the LEDC peripheral configuration
    LEDbubble_ledc_init();
    
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

}
