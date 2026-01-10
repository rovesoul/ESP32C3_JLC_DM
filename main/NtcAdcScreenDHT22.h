#ifndef NTC_ADC_SCREEN_DHT22_H
#define NTC_ADC_SCREEN_DHT22_H

#include <stdint.h>

// PID结构体定义
typedef struct {
    float target_temp;
    float kp, ki, kd;
    float integral;
    float last_error;
    float filtered_derivative;  // 新增：滤波后的微分值
    float output;
    uint32_t pwm_duty;
} pid_controller_t;

// 声明全局变量
extern pid_controller_t heater_pid;

#endif // NTC_ADC_SCREEN_DHT22_H