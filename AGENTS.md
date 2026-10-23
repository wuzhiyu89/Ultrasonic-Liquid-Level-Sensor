# AGENTS.md — G031（N32G031 + LLCC68）

基于国民技术 **N32G031** 的小型固件工程，带 LLCC68 LoRa 模块。**唯一可构建的是 Keil MDK-ARM 工程**（命令行也只接了这套）。

- Keil 工程：`MDK-ARM\Interrupt.uvprojx`（目标名 `N32G031`）
- IAR 工程：`EWARM\Interrupt.eww` —— ⚠️ **不可用，勿依赖**，见下面【IAR 工程状态】

## IAR 工程状态（2026-10-22 复核，仍未修）

`EWARM\Interrupt.ewp` 与当前代码**严重不同步**，打开也编不过：

- 文件列表只有 `main.c`、`n32g031_it.c` 和 4 个库文件；`peripherals/app|bsp|sensor|radio` 全缺，链接必然一堆 undefined。
- 没有定义 `SYSCLK_FREQ=8000000`（Keil 工程有）。`SystemCoreClock` 会落到默认 48 MHz，
  于是 SysTick 延时和 TIM3 节拍全变成 **6 倍**：所有超时、接收窗、`LLCC68DelayMs` 都会膨胀 6 倍。

结论：**改动只在 Keil 工程里做**。真要恢复 IAR，必须补齐文件列表 + 加 `SYSCLK_FREQ` 宏，
并在台架上重新量 0x04 配对帧的空口间隔（应≈5 s）确认时基没错。

## 目录职责

| 目录 | 内容 |
|---|---|
| `src/` | `main.c`、`n32g031_it.c` |
| `inc/` | `main.h`、`n32g031_it.h` |
| `peripherals/` | 应用与外设层：`LLCC68N32G031-board.c`、`app_lora.c`、`app_cmd.c`、`app_dev_param.c` |
| `firmware/` | **厂商 CMSIS + N32G031 库 —— 不要修改** |
| `MDK-ARM/` `EWARM/` | 两套 IDE 工程与构建产物 |
| `相关资料/` | 参考文档，只读 |

## 构建

用 MCP 工具：`firmware_build` 目标名 `G031`。等价命令：

```powershell
$log = "$env:TEMP\G031-build.log"
$p = Start-Process -FilePath 'D:\Keil_v5\UV4\UV4.exe' `
     -ArgumentList @('-j0','-b','C:\Users\wzy886\Desktop\G031\MDK-ARM\Interrupt.uvprojx','-o',$log) `
     -WorkingDirectory 'C:\Users\wzy886\Desktop\G031\MDK-ARM' -Wait -PassThru
Get-Content -LiteralPath $log -Tail 100
exit $p.ExitCode
```

- **先关闭 uVision 里打开的 Interrupt 工程**。
- 以日志里的 `0 Error(s)` 为准。
- 烧录：`firmware_flash` 目标 `G031`。

## 改动规则

- 改完必须编译一次再下结论。
- 新增/删除源文件只改 Keil 工程的 `Interrupt.uvprojx` 即可（IAR 工程已作废，见上）。
- 不要动 `firmware/` 与 `相关资料/`。
- LLCC68 相关改动（`peripherals/`）说明对 SPI 时序、忙检测（BUSY 引脚）、中断/DIO 处理的影响。
  ⚠️ `LLCC68WaitOnBusy()` 有 50 ms 硬超时（超时后 `LLCC68GetBusyTimeoutCount()` +1），
  别改回无超时的死循环。
- 本工程**不能开 IWDG**：它挂在 LSI 上、STOP 期间照样计数，睡 10 分钟必被复位；
  N32G031 的选项字节里也没有"IWDG 在 STOP 冻结"这一位。阻塞等待一律自带超时。
- 源码为 UTF-8；PowerShell 读写加 `-Encoding utf8`（详见用户级 `AGENTS.md`）。
