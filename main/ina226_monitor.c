#include "ina226_monitor.h"

#include <inttypes.h>
#include <math.h>
#include <string.h>
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#define INA226_I2C_ADDRESS          0x40
#define INA226_I2C_SPEED_HZ         400000
#define INA226_REG_CONFIG           0x00
#define INA226_REG_SHUNT_VOLTAGE    0x01
#define INA226_REG_BUS_VOLTAGE      0x02
#define INA226_REG_POWER            0x03
#define INA226_REG_CURRENT          0x04
#define INA226_REG_CALIBRATION      0x05

#define INA226_SHUNT_OHM            0.005f
#define INA226_MAX_CURRENT_A        16.384f
#define INA226_CURRENT_LSB_A        (INA226_MAX_CURRENT_A / 32768.0f)
#define INA226_CALIBRATION_VALUE    2048
#define INA226_CONFIG_VALUE         0x4527
#define INA226_NVS_NAMESPACE        "ina226_cfg"
#define INA226_NVS_CORRECTION_KEY   "curr_corr"
#define INA226_OFFLINE_THRESHOLD    5

extern i2c_master_bus_handle_t oled_bus_handle;
extern bool is_OPEN;
extern bool FAN_IS_RUNNING;

static const char *TAG = "ina226";
static i2c_master_dev_handle_t s_ina226_dev = NULL;
static SemaphoreHandle_t s_lock = NULL;
static int64_t s_last_sample_time_us = 0;
static int64_t s_last_session_sample_time_us = 0;
static bool s_last_system_open = false;
static bool s_session_active = false;
static ina226_status_t s_status = {
    .ready = false,
    .current_correction = 1.0f,
};

static esp_err_t ina226_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t data[2] = {0};
    esp_err_t err = i2c_master_transmit_receive(s_ina226_dev, &reg, 1, data, sizeof(data), pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        return err;
    }

    *value = ((uint16_t)data[0] << 8) | data[1];
    return ESP_OK;
}

static esp_err_t ina226_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t data[3] = {
        reg,
        (uint8_t)(value >> 8),
        (uint8_t)(value & 0xff),
    };

    return i2c_master_transmit(s_ina226_dev, data, sizeof(data), pdMS_TO_TICKS(100));
}

static void ina226_load_correction(void)
{
    nvs_handle_t handle;
    float correction = 1.0f;

    esp_err_t err = nvs_open(INA226_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_OK) {
        size_t size = sizeof(correction);
        err = nvs_get_blob(handle, INA226_NVS_CORRECTION_KEY, &correction, &size);
        nvs_close(handle);
    }

    if (err != ESP_OK || !isfinite(correction) || correction <= 0.0f || correction > 100.0f) {
        correction = 1.0f;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_status.current_correction = correction;
        xSemaphoreGive(s_lock);
    }

    ESP_LOGI(TAG, "current correction loaded: %.4f", correction);
}

static esp_err_t ina226_save_correction(float correction)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(INA226_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }

    err = nvs_set_blob(handle, INA226_NVS_CORRECTION_KEY, &correction, sizeof(correction));
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t ina226_monitor_init(void)
{
    if (oled_bus_handle == NULL) {
        ESP_LOGE(TAG, "OLED I2C bus is not ready");
        return ESP_ERR_INVALID_STATE;
    }

    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
        if (s_lock == NULL) {
            return ESP_ERR_NO_MEM;
        }
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = INA226_I2C_ADDRESS,
        .scl_speed_hz = INA226_I2C_SPEED_HZ,
        .flags.disable_ack_check = false,
    };

    esp_err_t err = i2c_master_bus_add_device(oled_bus_handle, &dev_cfg, &s_ina226_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "add INA226 device failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_RETURN_ON_ERROR(ina226_write_reg(INA226_REG_CONFIG, INA226_CONFIG_VALUE), TAG, "write config failed");
    ESP_RETURN_ON_ERROR(ina226_write_reg(INA226_REG_CALIBRATION, INA226_CALIBRATION_VALUE), TAG, "write calibration failed");
    ina226_load_correction();

    ESP_LOGI(TAG, "INA226 initialized addr=0x%02x shunt=%.3f ohm current_lsb=%.6f A calibration=%u",
             INA226_I2C_ADDRESS, INA226_SHUNT_OHM, INA226_CURRENT_LSB_A, INA226_CALIBRATION_VALUE);
    return ESP_OK;
}

ina226_status_t ina226_get_status(void)
{
    ina226_status_t snapshot = {0};
    if (s_lock != NULL && xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        snapshot = s_status;
        xSemaphoreGive(s_lock);
    }
    return snapshot;
}

esp_err_t ina226_set_current_correction_from_reference(float reference_current_ma)
{
    if (!isfinite(reference_current_ma) || reference_current_ma <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_lock == NULL || xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_INVALID_STATE;
    }

    float raw_current_ma = fabsf(s_status.raw_current_ma);
    xSemaphoreGive(s_lock);

    if (raw_current_ma < 0.1f) {
        return ESP_ERR_INVALID_STATE;
    }

    float correction = reference_current_ma / raw_current_ma;
    if (!isfinite(correction) || correction <= 0.0f || correction > 100.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ina226_save_correction(correction);
    if (err != ESP_OK) {
        return err;
    }

    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_status.current_correction = correction;
        s_status.current_ma = s_status.raw_current_ma * correction;
        s_status.power_w = s_status.bus_voltage_v * (s_status.current_ma / 1000.0f);
        xSemaphoreGive(s_lock);
    }

    ESP_LOGI(TAG, "current correction updated: reference=%.3f mA raw=%.3f mA factor=%.4f",
             reference_current_ma, raw_current_ma, correction);
    return ESP_OK;
}

void ina226_monitor_task(void *pvParameter)
{
    (void)pvParameter;

    while (true) {
        if (s_ina226_dev == NULL) {
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        uint16_t bus_raw = 0;
        uint16_t current_raw_u = 0;
        uint16_t power_raw = 0;
        esp_err_t err = ina226_read_reg(INA226_REG_BUS_VOLTAGE, &bus_raw);
        if (err == ESP_OK) {
            err = ina226_read_reg(INA226_REG_CURRENT, &current_raw_u);
        }
        if (err == ESP_OK) {
            err = ina226_read_reg(INA226_REG_POWER, &power_raw);
        }

        bool system_open = is_OPEN;
        bool fan_running = FAN_IS_RUNNING;
        if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (err == ESP_OK) {
                int16_t current_raw = (int16_t)current_raw_u;
                float bus_voltage_v = bus_raw * 0.00125f;
                float raw_current_ma = current_raw * INA226_CURRENT_LSB_A * 1000.0f;
                float current_ma = raw_current_ma * s_status.current_correction;
                float power_w = bus_voltage_v * (current_ma / 1000.0f);
                int64_t now_us = esp_timer_get_time();

                if (s_last_sample_time_us > 0 && power_w > 0.0f) {
                    float elapsed_hours = (float)(now_us - s_last_sample_time_us) / 3600000000.0f;
                    if (elapsed_hours > 0.0f && elapsed_hours < 1.0f) {
                        float delta_wh = power_w * elapsed_hours;
                        s_status.total_energy_wh += delta_wh;
                    }
                }

                if (system_open && !s_last_system_open) {
                    s_status.session_energy_wh = 0.0f;
                    s_last_session_sample_time_us = now_us;
                    s_session_active = true;
                } else if (s_session_active && (system_open || fan_running) &&
                           s_last_session_sample_time_us > 0 && power_w > 0.0f) {
                    float elapsed_hours = (float)(now_us - s_last_session_sample_time_us) / 3600000000.0f;
                    if (elapsed_hours > 0.0f && elapsed_hours < 1.0f) {
                        float delta_wh = power_w * elapsed_hours;
                        s_status.session_energy_wh += delta_wh;
                    }
                }
                if (s_session_active) {
                    s_last_session_sample_time_us = now_us;
                    if (!system_open && !fan_running) {
                        s_session_active = false;
                        s_last_session_sample_time_us = 0;
                    }
                }

                s_last_sample_time_us = now_us;
                s_last_system_open = system_open;

                s_status.ready = true;
                s_status.consecutive_failures = 0;
                s_status.bus_voltage_v = bus_voltage_v;
                s_status.raw_current_ma = raw_current_ma;
                s_status.current_ma = current_ma;
                s_status.power_w = power_w;
                (void)power_raw;
            } else {
                s_status.consecutive_failures++;
                s_last_sample_time_us = 0;
                if (s_status.consecutive_failures >= INA226_OFFLINE_THRESHOLD) {
                    s_status.ready = false;
                }
            }
            xSemaphoreGive(s_lock);
        }

        if (err != ESP_OK) {
            uint32_t failures = 0;
            if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
                failures = s_status.consecutive_failures;
                xSemaphoreGive(s_lock);
            }
            if (failures == 1 || failures % 30 == 0) {
                ESP_LOGW(TAG, "read failed: %s, consecutive=%" PRIu32, esp_err_to_name(err), failures);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
