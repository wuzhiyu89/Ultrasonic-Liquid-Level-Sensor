# Ultrasonic-Liquid-Level-Sensor

基于 N32G031 的低功耗超声波液位传感器节点。超声波测距读取液位，经 LoRa（SX1261）上报，支持配对/解绑，STOP 模式待机功耗低至 µA 级。

## 硬件

- MCU：N32G031F8（Cortex-M0，64KB Flash / 8KB RAM）
- 超声波液位模块（USART1 / PA10，9600 baud）
- LoRa：SX1261 / LLCC68（SPI）
- 探针电路（水浸检测）
- 外设域由 PB7 控制的高边 P-MOS 统一供电
- 最小系统板：N32G031C8L7-STB V1.0

## 主要功能

- 超声波测距并上报液位
- LoRa 配对（0x04）/ 解绑（0x05）
- 参数设置（0x01）/ 液位上报（0x02）/ ACK（0x80/0x81）
- STOP 模式低功耗，RTC 唤醒后自动恢复外设并测距/收发

## 通信协议

帧格式：`0xAA 0x55 | 版本 | 网关地址(3B) | 设备地址(3B) | 功能码 | 数据长度 | 数据 | CRC16-Modbus(2B)`

CRC 覆盖范围：从版本字段到数据末尾。

| 功能码 | 含义 |
|---|---|
| 0x01 | 设置参数 |
| 0x02 | 液位上报 |
| 0x04 | 配对 |
| 0x05 | 解绑 |
| 0x80 | ACK-OK |
| 0x81 | ACK-ERR |

## 目录结构

```
demo01/
├── src/              # main.c、中断
├── inc/              # 公共头文件
├── peripherals/
│   ├── app/          # 业务层：lora / sensor / lowpower / pair / cmd ...
│   ├── bsp/          # 板级驱动（GPIO / 电源 / 超声波）
│   ├── common/       # 通用工具
│   ├── log/          # 日志输出
│   ├── radio/        # LoRa 射频（SX1261 / LLCC68 / delay）
│   └── sensor/       # 传感器抽象
├── MDK-ARM/          # Keil 工程（Interrupt.uvprojx）
└── EWARM/            # IAR 工程（Interrupt.ewp）
```

## 编译

- Keil MDK-ARM 5.26+：打开 `MDK-ARM/Interrupt.uvprojx`
- IAR EWARM：打开 `EWARM/Interrupt.eww`
- 需自行准备 Nations N32G031 标准固件库（本仓库仅含本项目代码）
- 通过 `src/main.c` 中的 `MAIN_MODE` 宏切换运行模式（默认 3 = 配对测试）

## 低功耗

STOP 模式下外设域通过 PB7 释放为高阻态实现 µA 级待机，唤醒后驱动低 + 延时等外设 POR + 重新初始化。详见 [低功耗方案A经验总结.md](低功耗方案A经验总结.md)。

## 关键约束

- PB7 禁止主动驱动高（会触发整板 POR/BOR 复位循环）
- PA9（超声波 TRIG）进 STOP 前必须置低
- 收到 0x05 解绑后立即回 0x80 ACK → 清网关地址为 FFFFFF → 进 STOP
