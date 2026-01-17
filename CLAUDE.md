# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is an **ESP32-C3 based smart temperature control system** for drying applications. It integrates multiple sensors (NTC thermistor, DHT22), actuators (heating element, fan), and user interfaces (OLED display, web interface) to create a closed-loop PID temperature control system with WiFi connectivity.

## Build and Development Commands

```bash
# Set up ESP-IDF environment (required before any commands)
. $HOME/esp/esp-idf/export.sh

# Configure project (ESP-IDF v5.x, target: ESP32-C3)
idf.py menuconfig

# Build the project
idf.py build

# Flash to device (adjust port as needed)
idf.py -p /dev/ttyUSB0 flash

# Monitor serial output
idf.py -p /dev/ttyUSB0 monitor

# Combined flash and monitor
idf.py -p /dev/ttyUSB0 flash monitor
```

## Architecture

### Component Structure

The main application is organized as a single ESP-IDF component in `main/` with the following modules:

- **[NtcAdcScreenDHT22.c](main/NtcAdcScreenDHT22.c)** - Main application logic (29KB), FreeRTOS task creation, PID controller
- **[ntc.c](main/ntc.c)** - NTC temperature sensor driver (ADC-based)
- **[dht22.c](main/dht22.c)** - DHT22 temperature/humidity sensor driver
- **[OLED.c](main/OLED.c)** - SSD1306 OLED display driver (I2C, 128x64)
- **[OLED_Data.c](main/OLED_Data.c)** - OLED font and graphic data
- **[simple_wifi_sta.c](main/simple_wifi_sta.c)** - WiFi Station mode + HTTP server with embedded web interface

### FreeRTOS Task Architecture

All tasks are pinned to **Core 0** with specific priorities:

| Task | Priority | Stack | Cycle | Purpose |
|------|----------|-------|-------|---------|
| led_breath | 2 | 2048B | - | Breathing LED effect |
| dht22 | 3 | 2048B | 2s | Read DHT22 sensor |
| oled_disp | 3 | 3072B | 500ms | Refresh OLED display |
| pid_ctrl | 4 | 4096B | 200ms | PID temperature control |

### Hardware Peripherals

**GPIO Assignments** (see [README.md](README.md) for full details):
- OLED I2C: SCL=GPIO 5, SDA=GPIO 4 (address 0x78)
- Breathing LED: GPIO 2 (PWM, 1000Hz, 13-bit)
- Heating element: GPIO 10 (PWM, 500Hz, 10-bit, range 0-10000)
- Fan: GPIO 6 (digital output)
- DHT22: GPIO 0

**PWM Channels**:
- LEDC_CHANNEL_0: Breathing LED
- LEDC_CHANNEL_1: Heating element control

### PID Control System

The system implements a PID controller with filtered derivative:

- **Control variable**: NTC temperature (heating element temperature)
- **Setpoint**: Configurable target temperature (default 35.0°C)
- **Output**: PWM duty cycle (0-10000) to heating element
- **Control cycle**: 200ms
- **Output limits**: 0-10000 (maps to 0-100% PWM)
- **Safety limits**: 0-95°C operating range

**Control Logic**:
- When `is_OPEN` (system enabled): PID controls heating + fan on
- When disabled: Heating off, fan turns off below 35°C

### Display Modes

Two display modes are available via `DISPLAY_MODE` macro in [NtcAdcScreenDHT22.c](main/NtcAdcScreenDHT22.c):
- **Mode 0**: Temperature curve (56 data points) + text info
- **Mode 1**: Gauge display (0-100°C range) + text info

### Web Interface

The embedded web interface ([index.html](main/index.html)) is served by the HTTP server and provides:
- PID parameter adjustment (P, I, D, target temperature)
- System ON/OFF control
- Real-time monitoring with Chart.js graphs (temperature and humidity curves)
- Current PWM output percentage

## WiFi Configuration

WiFi credentials are configured in [simple_wifi_sta.c](main/simple_wifi_sta.c). The device operates in Station mode and auto-connects on boot.

## Key Files to Understand

- **[main/NtcAdcScreenDHT22.h](main/NtcAdcScreenDHT22.h)** - PID controller structure (`pid_controller_t`), global state variables
- **[main/NtcAdcScreenDHT22.c](main/NtcAdcScreenDHT22.c)** - `app_main()` initialization, all FreeRTOS task implementations
- **[sdkconfig](sdkconfig)** - ESP-IDF configuration (ESP32-C3 specific, include paths)
- **[main/CMakeLists.txt](main/CMakeLists.txt)** - Component dependencies (embeds index.html as text file)

## Development Notes

- Target platform: **ESP32-C3** (RISC-V architecture)
- Framework: **ESP-IDF v5.5.2**
- The project contains backup files in [backup/](backup/) directory from iterative development
- Main application file is large (29KB) - consider modularization for new features
- All sensor drivers and OLED driver use custom implementations (not ESP-IDF component drivers)
