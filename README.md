# NtcAdcScreenDHT22

基于ESP32-C3的智能温控烘干项目,集成了NTC温度传感器、DHT22温湿度传感器、OLED显示屏、加热片、风扇和呼吸灯,支持PID温度控制和Web远程配置。

## 功能特性

- [x] NTC温度监测 (ADC采集)
- [x] DHT22温湿度监测
- [x] PID温度控制
- [x] OLED显示屏 (温度曲线/仪表盘模式)
- [x] 加热片PWM控制
- [x] 风扇自动控制
- [x] 呼吸灯状态指示
- [x] WiFi STA模式连接
- [x] Web配置界面
- [x] 温度曲线图表显示
- [x] 远程PID参数调整
- [x] 远程开关控制
- [ ] 阶段优化冗余代码
- [ ] 通风换气
- [ ] 联网MQTT数据显示

## 硬件配置

### GPIO引脚分配

| 功能模块 | GPIO引脚 | 说明 |
|---------|---------|------|
| OLED SCL | GPIO 5 | I2C时钟线 |
| OLED SDA | GPIO 4 | I2C数据线 |
| 呼吸灯 | GPIO 2 | LED PWM输出 |
| 加热片 | GPIO 10 | 加热片PWM控制 |
| 风扇 | GPIO 6 | 风扇开关控制 |
| DHT22 | GPIO 0 | 温湿度传感器数据线 |

### OLED显示屏
- 型号: SSD1306 (128x64)
- 通信方式: I2C
- 地址: 0x78
- 速度: 400kHz

### PWM配置

| PWM通道 | 用途 | 频率 | 分辨率 | 最大占空比 |
|---------|------|------|--------|-----------|
| LEDC_CHANNEL_0 | 呼吸灯 | 1000Hz | 13位 | 8191 |
| LEDC_CHANNEL_1 | 加热片 | 500Hz | 10位 | 10000 |

## PID参数配置

默认PID参数:
- 目标温度 (TARGET_TEMP): 35.0°C
- 比例增益 (PID_KP): 200.0
- 积分增益 (PID_KI): 180.0
- 微分增益 (PID_KD): 10.0

安全限制:
- 最高温度限制: 95.0°C
- 最低温度限制: 0.0°C
- PID控制周期: 200ms
- 死区范围: ±0.1°C

## 显示模式

项目支持两种OLED显示模式,通过修改 `DISPLAY_MODE` 宏定义切换:

### 模式0: 曲线+文字 (DISPLAY_MODE=0)
- 左侧: 实时温度曲线 (56个数据点)
- 右侧: 显示目标温度、PID参数、NTC温度、DHT温湿度、PWM百分比

### 模式1: 仪表盘+文字 (DISPLAY_MODE=1)
- 左侧: 仪表盘显示 (0-100°C范围,带目标温度标记)
- 右侧: 同模式0的文字信息

## Web配置界面

### 访问方式
连接ESP32-C3 WiFi后,通过浏览器访问设备IP地址。

### 功能
1. **PID参数配置**
   - P值调整
   - I值调整
   - D值调整
   - 目标温度设置

2. **开关控制**
   - 启动/关闭加热烘干功能
   - 按钮颜色指示状态 (红色=关闭, 绿色=运行)

3. **实时数据监控**
   - 当前P、I、D值
   - 目标温度
   - NTC温度
   - DHT温度和湿度
   - PWM百分比
   - 温度曲线图表
   - 湿度曲线图表

## 编译和烧录

### 环境要求
- ESP-IDF v5.x
- 支持 CMake 3.16+

### 编译步骤

```bash
# 设置ESP-IDF环境变量
. $HOME/esp/esp-idf/export.sh

# 配置项目
idf.py menuconfig

# 编译
idf.py build

# 烧录
idf.py -p /dev/ttyUSB0 flash

# 查看串口日志
idf.py -p /dev/ttyUSB0 monitor
```

### 一键操作
```bash
# 编译+烧录+监控
idf.py -p /dev/ttyUSB0 flash monitor
```

## WiFi配置

项目使用Station模式,需要配置SSID和密码:
1. 在 `simple_wifi_sta.c` 中修改WiFi凭据
2. 或通过menuconfig配置
3. 设备启动后自动连接WiFi

## 文件结构

```
main/
├── NtcAdcScreenDHT22.c    # 主程序文件 (任务创建、PID控制)
├── NtcAdcScreenDHT22.h    # 主程序头文件
├── ntc.c                  # NTC温度传感器驱动
├── ntc.h                  # NTC头文件
├── dht22.c                # DHT22温湿度传感器驱动
├── dht22.h                # DHT22头文件
├── OLED.c                 # OLED显示屏驱动
├── OLED.h                 # OLED头文件
├── OLED_Data.c            # OLED字体/图形数据
├── OLED_Data.h            # OLED数据头文件
├── simple_wifi_sta.c      # WiFi STA模式实现
├── simple_wifi_sta.h      # WiFi头文件
├── index.html             # Web配置界面
└── CMakeLists.txt         # 构建配置
```

## FreeRTOS任务

系统启动后创建以下任务:

| 任务名称 | 优先级 | 栈大小 | 核心绑定 | 功能描述 |
|---------|-------|--------|---------|---------|
| led_breath | 2 | 2048 | Core 0 | 呼吸灯控制 |
| dht22 | 3 | 2048 | Core 0 | DHT22温湿度读取 (2秒周期) |
| pid_ctrl | 4 | 4096 | Core 0 | PID温度控制 (200ms周期) |
| oled_disp | 3 | 3072 | Core 0 | OLED屏幕刷新 (500ms周期) |

## 工作流程

1. **系统初始化**
   - NVS初始化
   - WiFi STA模式初始化
   - 风扇GPIO配置
   - 呼吸灯PWM初始化
   - 加热片PWM初始化
   - PID控制器初始化
   - NTC温度传感器初始化
   - DHT22传感器配置
   - OLED显示屏初始化

2. **温度控制逻辑**
   - NTC传感器实时采集温度
   - 根据开关状态 (is_OPEN) 决定控制方式
   - 开启状态: PID控制加热片+风扇开启
   - 关闭状态: 加热片关闭,温度<35°C时关闭风扇

3. **安全保护**
   - 温度超过95°C时强制关闭加热片
   - PID输出限制在0-10000范围内

## 技术栈

- **MCU**: ESP32-C3
- **框架**: ESP-IDF v5.x
- **RTOS**: FreeRTOS
- **显示**: SSD1306 OLED (128x64)
- **通信**: WiFi STA + HTTP Server
- **前端**: HTML + CSS + JavaScript (Chart.js)

## 开发计划

- [ ] 优化冗余代码
- [ ] 增加通风换气功能
- [ ] 实现MQTT数据上报
- [ ] 温度数据历史记录
- [ ] 报警功能 (超温/低温)

## 许可证

本项目采用 Unlicense 或 CC0-1.0 许可证。

## 作者

Espressif Systems (Shanghai) CO LTD
