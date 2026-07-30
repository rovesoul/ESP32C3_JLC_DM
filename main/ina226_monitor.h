#ifndef INA226_MONITOR_H
#define INA226_MONITOR_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    bool ready;
    float bus_voltage_v;
    float current_ma;
    float power_w;
    float raw_current_ma;
    float current_correction;
    float session_energy_wh;
    float total_energy_wh;
    uint32_t consecutive_failures;
} ina226_status_t;

esp_err_t ina226_monitor_init(void);
void ina226_monitor_task(void *pvParameter);
ina226_status_t ina226_get_status(void);
esp_err_t ina226_set_current_correction_from_reference(float reference_current_ma);

#endif
