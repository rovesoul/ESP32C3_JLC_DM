# NtcAdcScreenDHT22

基于 ESP32-C3 的智能温控烘干控制器。项目集成 NTC 温度采集、DHT22 温湿度、SSD1306 OLED、加热片 PWM、风扇 PWM、INA226 电压/电流/功率监测，以及 Web 配置与实时图表。

当前工程已按 ESP-IDF v6.0 和 4MB flash 配置，网页文件通过 `EMBED_TXTFILES` 编译进固件。

## 功能

- NTC 温度采集，用于 PID 主控温
- DHT22 温湿度显示，带失联状态和旧值提示
- PID 参数、目标温度、风扇转速、定时器网页配置
- 加热片 PWM 输出
- 风扇 PWM 输出，系统关闭后温度高于 35°C 时继续散热
- OLED 本地显示，当前默认半圆仪表盘模式
- WiFi 配网页，无需硬编码 WiFi
- INA226 可选接入，自动在线/离线判断
- INA226 电压、电流、功率显示
- INA226 电流校正，倍率保存到 NVS
- 电流显示自动切换单位：超过 1200mA 显示为 A，保留 2 位小数
- Web 图表：温度、湿度、电压/电流、上电累计能量
- 页签 favicon：圆圈内红色 H

- [ ] 阶段优化冗余代码
- [ ] 通风换气
- [ ] 联网 MQTT 数据显示

## 硬件连接

| 功能 | 引脚/参数 | 说明 |
| --- | --- | --- |
| OLED SCL | GPIO5 | I2C 时钟 |
| OLED SDA | GPIO4 | I2C 数据 |
| INA226 SCL | GPIO5 | 与 OLED 共用 I2C |
| INA226 SDA | GPIO4 | 与 OLED 共用 I2C |
| INA226 地址 | 0x40 | 默认地址 |
| INA226 采样电阻 | 0.005 ohm | 5mΩ |
| 呼吸灯 | GPIO2 | LEDC PWM |
| 加热片 | GPIO10 | LEDC PWM |
| 风扇 | GPIO6 | LEDC PWM |
| DHT22 | GPIO0 | 数据线，内部上拉已启用 |
| 配网按钮 | GPIO9 | 长按进入/清除配网 |

OLED 使用 SSD1306 128x64，I2C 速度 400kHz。

## PWM 配置

| 输出 | LEDC 通道 | 频率 | 分辨率 |
| --- | --- | --- | --- |
| 呼吸灯 | LEDC_CHANNEL_0 | 1000Hz | 13 位 |
| 加热片 | LEDC_CHANNEL_1 | 500Hz | 10 位 |
| 风扇 | LEDC_CHANNEL_2 | 500Hz | 10 位 |

风扇配置值会限制在 0-95%。低于 10% 会关闭，10-20% 会提升到 20%，避免极低占空比下 MOSFET/风扇状态不稳定。

## 控温逻辑

默认参数：

- 目标温度：35.0°C
- P：200.0
- I：180.0
- D：10.0
- PID 周期：200ms
- 最高温度保护：95°C

运行状态：

- 系统开启：PID 控制加热片，风扇按网页配置转速运行
- 系统关闭：加热片关闭
- 系统关闭且 NTC < 35°C：风扇关闭
- 系统关闭且 NTC >= 35°C：风扇继续按配置转速散热
- 上电时强制 `is_OPEN=false`，并写入 NVS，避免断电恢复后自动加热

## 工作流程

### 1. 上电初始化

系统启动后先初始化 NVS，然后创建 DHT22 和 OLED 互斥锁，初始化 OLED。OLED 要先于 WiFi 初始化，因为首次配网时需要在屏幕上显示配网热点和地址。

之后依次初始化：

- INA226：与 OLED 共用 I2C 总线，初始化失败时只记录警告，不影响主控温功能
- 呼吸灯 PWM
- 风扇 PWM，初始 duty 为 0
- 加热片 PWM，初始 duty 为 0
- PID 控制器
- NTC ADC
- DHT22 GPIO

### 2. WiFi 和配网

初始化完基础硬件后进入 WiFi 初始化流程：

- 如果 NVS 中没有 WiFi 配置，设备进入 AP 配网模式
- OLED 显示配网热点、密码和 `192.168.4.1`
- 配网成功后保存 WiFi SSID/密码到 NVS，并重启
- 如果已有 WiFi 配置，直接进入 STA 模式连接路由器

WiFi 连接成功并拿到 IP 后启动 HTTP Server，注册首页、`/values`、`/config`、`/toggle`、`/toggle/status`、`/ina226/calibrate` 等接口。

### 3. 上电安全状态

WiFi 初始化阶段会强制设置：

- `is_OPEN=false`
- `toggle_state=false` 写入 NVS
- 加载 PID、风扇转速、定时器配置，但不立即应用风扇 PWM

这段逻辑是为了让网页状态、NVS 状态和芯片运行状态保持一致，避免断电恢复后自动加热。

### 4. 任务启动

初始化完成后创建 FreeRTOS 任务：

- `led_breath`：呼吸灯效果
- `dht22`：每 2 秒读取 DHT22，成功才更新温湿度
- `pid_ctrl`：每 200ms 读取 NTC 并执行控温
- `oled_disp`：每 500ms 刷新 OLED
- `ina226`：如果 INA226 初始化成功，每 1 秒采样一次电压/电流/功率

### 5. 控温运行

网页点击启动后，`is_OPEN=true`：

- 记录系统运行开始时间
- 如果配置了定时器，则启动定时器
- PID 根据 NTC 温度计算加热片 PWM
- 风扇按网页配置转速运行
- 如果超过目标温度 5°C，触发超温保护，加热片关闭，风扇全速
- 如果超过 95°C，触发极限保护，加热片关闭，系统自动关闭，风扇全速

网页点击关闭后，`is_OPEN=false`：

- 停止定时器
- 清空运行时间
- 加热片关闭
- 如果 NTC 仍高于 35°C，风扇继续散热
- 当 NTC 低于 35°C，风扇停止

### 6. 数据刷新和图表

网页每秒请求 `/values`，从固件获取当前状态。图表数据主要在浏览器内存中维护，刷新网页后图表会重新开始，但设备端 RAM 中的上电累计能量不会因为刷新网页而清零。

INA226 相关区域只有模块在线时显示。电压和电流在前端进行自适应滤波后显示和绘图。

### 7. 能量统计

INA226 任务每秒计算一次功率积分：

- 上电累计能量：只要 INA226 在线且功率为正，就从本次上电开始累计到 RAM
- 本次能量：从系统启动开始累计；关闭后如果风扇还在运行，继续累计；风扇停止后本轮结束

这两个能量值都不写 NVS，断电或重启后自然清零。

## INA226 说明

INA226 是可选模块。固件启动时会尝试初始化 INA226：

- 接入成功：网页显示电压、电流、功率、电流校正、能量和 INA226 图表
- 未接入或连续读取失败：网页隐藏 INA226 相关区域
- 后端连续失败 5 次后判定离线，避免偶发 I2C 失败导致页面跳动

前端对 INA226 电压/电流做了自适应滤波：

- 小变化使用平滑滤波，降低抖动
- 大跳变使用快速跟随，避免打开电源后曲线迟钝
- 当前参数：基础 alpha 0.28，快速 alpha 0.82，电压快速阈值 0.8V，电流快速阈值 4mA

### 能量定义

- 本次能量：从点击“启动”开始累计；点击“关闭”后，如果风扇还在转，继续累计；直到风扇停止，这一轮结束。下一次启动时重新清零。
- 上电累计能量：从 ESP32 本次上电开始累计，断电或重启后清零。

上电累计能量只存放在 RAM 中，不写 NVS，不消耗 flash 擦写寿命。网页刷新后数值还在，是因为 ESP32 没有重启，RAM 变量仍在。

## DHT22 失联处理

DHT22 只有读取成功才更新温湿度。连续失败后：

- 后端 `dhtAvailable=false`
- 如果曾经读到过有效值，网页显示旧值并标红
- 如果从未读到有效值，网页显示 `DHT失联`
- 日志做了限频，避免超时日志刷屏

## Web 页面

访问设备 IP 即可打开网页。

页面包含：

- 系统启动/关闭
- PID 参数设置
- 目标温度设置
- 风扇转速设置
- 定时器设置
- INA226 电流校正
- 当前状态卡片
- 温度曲线
- 湿度曲线
- 电压/电流曲线
- 上电累计能量曲线

浏览器缓存问题：

- Safari 强制刷新：`Option + Cmd + R`
- Edge/Chrome 强制刷新：`Cmd + Shift + R`

HTTP 首页响应已设置 `Cache-Control: no-store`，但固件更新后仍建议强刷一次。

## WiFi 配网

首次没有 WiFi 配置时，设备会进入 AP 配网模式：

- AP 名称：`ESP32-Setup_xxxxxx`
- AP 密码：`12345678`
- 配网页地址：`192.168.4.1`
- OLED 会显示配网 SSID、密码和地址

配网成功后，WiFi SSID/密码保存到 NVS，设备重启并进入 STA 模式。

GPIO9 用于配网相关按键检测，具体交互见 `main/wifi_provisioning.c`。

## NVS 写入项

当前业务代码写入 NVS 的内容：

| 命名空间 | 键 | 内容 | 写入时机 |
| --- | --- | --- | --- |
| `wifi_config` | `wifi_ssid` | WiFi 名称 | 配网成功 |
| `wifi_config` | `wifi_password` | WiFi 密码 | 配网成功 |
| `storage` | `pid_values` | P/I/D/目标温度 | 保存配置 |
| `storage` | `fan_speed` | 风扇转速 | 保存配置 |
| `storage` | `timer_hours` | 定时时长 | 保存配置 |
| `storage` | `toggle_state` | 系统开关状态 | 上电强制关闭、网页开关 |
| `ina226_cfg` | `curr_corr` | INA226 电流校正倍率 | 应用电流校正 |

INA226 的本次能量、上电累计能量不写 NVS。

ESP-IDF WiFi 驱动自身也可能在 NVS 中保存内部 WiFi 数据。

## 分区表

当前使用自定义分区表 `partitions.csv`：

```csv
# Name,   Type, SubType, Offset,  Size,     Flags
nvs,      data, nvs,     0x9000,  0x6000,
phy_init, data, phy,     0xf000,  0x1000,
factory,  app,  factory, 0x10000, 0x300000,
```

说明：

- flash 大小：4MB
- app 分区：3MB
- 当前不是 OTA 双分区布局
- 如果以后要做网页 OTA，需要改成 `otadata + ota_0 + ota_1`，并首次通过 USB 串口烧录新分区表

## 编译和烧录

当前本机使用 ESP-IDF v6.0：

```bash
export PATH="$HOME/.espressif/tools/cmake/3.30.2/CMake.app/Contents/bin:$PATH"
. /Users/donghuibiao/ESPIDF/idfv6/v6.0/esp-idf/export.sh
idf.py build
```

烧录：

```bash
idf.py -p /dev/cu.usbmodem21401 flash
```

常见串口占用排查：

```bash
lsof /dev/cu.usbmodem21401 /dev/tty.usbmodem21401
```

如果被 `idf_monitor.py` 占用，先关闭 VSCode/终端监视器，或结束对应进程后再烧录。

如果遇到 ESP-IDF 路径或版本切换导致 CMake 继续引用旧 toolchain，删除 `build/` 或使用干净 build 目录后重新配置。

## 文件结构

```text
main/
├── NtcAdcScreenDHT22.c    # 主程序、任务、PID、风扇/加热 PWM、OLED 显示
├── NtcAdcScreenDHT22.h    # 全局结构和状态声明
├── ntc.c / ntc.h          # NTC ADC 采集
├── dht22.c / dht22.h      # DHT22 驱动
├── OLED.c / OLED.h        # SSD1306 OLED 驱动
├── OLED_Data.c/.h         # OLED 字库/图形数据
├── simple_wifi_sta.c/.h   # WiFi STA、HTTP Server、API
├── wifi_provisioning.c/.h # AP 配网
├── ina226_monitor.c/.h    # INA226 电压/电流/功率/能量
├── index.html             # 内嵌 Web 页面
└── CMakeLists.txt         # 组件构建配置
```

## FreeRTOS 任务

| 任务 | 优先级 | 栈大小 | 周期 | 功能 |
| --- | --- | --- | --- | --- |
| `led_breath` | 2 | 2048 | - | 呼吸灯 |
| `dht22` | 3 | 2048 | 2s | DHT22 读取 |
| `pid_ctrl` | 4 | 4096 | 200ms | PID 控温 |
| `oled_disp` | 3 | 3072 | 500ms | OLED 刷新 |
| `ina226` | 3 | 3072 | 1s | INA226 采样 |

INA226 初始化失败时不会创建 `ina226` 任务。

## 技术栈

- MCU：ESP32-C3
- 框架：ESP-IDF v6.0
- RTOS：FreeRTOS
- 显示：SSD1306 OLED
- 传感器：NTC、DHT22、INA226
- 网络：WiFi STA/AP 配网 + ESP HTTP Server
- 前端：HTML/CSS/JavaScript + Chart.js

## 后续计划

- OTA：规划双 OTA 分区并增加网页上传固件
- MQTT：接入远程监控
- 报警：超温、传感器离线、电流异常
- 图表：暂停/清空、原始值/滤波值切换
- 代码结构：继续拆分主文件中的显示和控制逻辑

## 作者

RoveSoul 2025.11-2026.07
