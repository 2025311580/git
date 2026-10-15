<!-- DO NOT EDIT - This part is automatically generated. -->

# Agent Guidelines

## CCStudio IDE Installation Directory

CCStudio IDE is installed at `C:/ti/ccs2101`. Save it as `{ccs-install-dir}` for the session — scripts and tools will need it.

## MANDATORY Pre-Task Requirement (DO NOT SKIP)

**CRITICAL - NO EXCEPTIONS**: Before ANY CCS/Texas Instruments-related task (even simple ones), you MUST read `C:/ti/ccs2101/ccs/theia/resources/ai/CCS.md`. This file includes information on how to interact with CCS as well as device-specific information (UART backchannel pins, LED setup, transmit best practices, etc.). 

Do NOT call any ccs-project, ccs-debug, ccs-sysconfig, or ccs-serial MCP tools until CCS.md has been read.


<!-- DO NOT EDIT - This part is automatically generated. -->

<!-- User instructions should be added below this line -->

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

MSPM0G3507 (ARM Cortex-M0+) 巡线小车工程。8 路灰度传感器（74HC4051 多路复用 + ADC）巡线，
四路直流电机带编码器闭环 PI 速度控制，OLED 显示运行状态。

## 架构（重要）

**速度环在 10ms 定时器中断里执行**（固定控制周期，官方实时控制模式），主循环只做非实时任务：

- `TIMER_0_INST_IRQHandler`（10ms）：读编码器计数 → 四路 PI 速度环 → PWM 输出 → 启动斜坡
  - 主循环的任何阻塞（传感器扫描 ≈1.7ms、OLED）都不影响控制周期
- 主循环：灰度传感器扫描 → `Track_Line()` 巡线算法 → 通过 volatile 变量发布目标速度
  （`ctrl_target_l/r`、`ctrl_enable`、`startup_ramp`）→ 按键检测 → `__WFI()` 睡眠省电
- OLED 只在状态切换时刷新，仅显示 `[STOP]` / `[RUN ]`

## 巡线算法（main.c → Track_Line）

1. **位置计算**：8 路二值化结果 × 权重表 `{-12,-6,-3,-1,1,3,6,12}` 加权平均 ×25
   （单传感器端点 = ±300，对应满偏），叠加 D 项（`TRACK_D_GAIN`）
   - D 项差分的是归一化后的 `position`（末尾 `/25` 抵消 ×25），**不是**原始加权和 `error`
     —— `error` 会随压线传感器个数跳变，制造虚假 D 冲击导致误转向
   - D 项再做一次两点平均（`filtered_d`）抑制单帧噪声
2. **转向强度**（分段映射，中间段陡峭，把车"钉"在中间两个传感器 S3/S4 上）：
   |位置| 0 → 0%（直行）；25（S3/S4 单路）→ 40%；75（S2/S5）→ 70%；150（S1/S6）→ 90%；≥300（S0/S7）→ 100%
3. **内外轮速度 = 直行速度 × 倍率**（turn 越大差速越大）：
   - 公式段：外轮 = 直行 ×(1 + turn×GAIN/10000)，内轮 = 直行 ×(1 − turn×DROP/10000)
   - **满矫正**（外轮 2.5×直行、内轮 0 制动 → 原地转向）不是硬切换，而是按权重 `w`
     从 |位置| 200→300 平滑混合过去（全白补救直接 w=100）。这样急转临界点不跳变，
     两端点值与原来的分支写法完全一致
4. **全白丢线保护**（`black_cnt == 0`）：
   - 按 `last_dir`（最后一次压线方向，居中时保持不清零）强制满矫正回拉，绝不直行
   - `LOST_LINE_TIMEOUT_MS` > 0 时超时改直行防原地打转（默认 0 = 不超时）
   - 关键依赖：`grayscale_sensor.c` 低对比度（8 路 max−min < 100）时归一化置"地面值"
     （`SENSOR_INVERT ? 0 : 1`），二值化全 0 → 才能正确触发丢线保护
   - ADC 超时返回 `0xFFFF` 时 `ScanAll` 沿用上一次有效值，不写进 `raw[]`
     （否则会把 rmax 抬到 65535，那一路被误判成"最白" = 凭空造线 → 满矫正急转）
5. **速度斜坡**：每 10ms 左右轮目标最多变化 `SPEED_RAMP`，防阶跃

## 调参指南

**唯一旋钮**：`STRAIGHT_SPEED`（main.c，直行速度，×20 = PWM，如 4 → 80 PWM，最大约 50 → 1000 PWM）

| 现象 | 调整 |
|---|---|
| 想更快/更慢 | `STRAIGHT_SPEED` 增减（唯一必调项） |
| 满矫正太猛甩头 | `TURN_FULL_OUTER` 25→20；或把混合起点提前/推后（`abs_pos > 200` 处） |
| 满矫正不够 | `TURN_FULL_OUTER` 25→30 |
| 直道蛇形摆动 | `TRACK_D_GAIN` 8→5，或 0~25 段映射 40→30 |
| 直道仍漂移 | 0~25 段映射 40→50 |
| 整体偏一侧（机械） | `LEFT_TRIM` 100→103 等 |
| 跑飞后转圈找不到线 | `LOST_LINE_TIMEOUT_MS` 设 300~500 |
| 线极性反了 | `SENSOR_INVERT`（grayscale_sensor.c）0↔1 |

其他可调：`SPEED_RAMP`（斜坡斜率）、`STARTUP_RAMP_*`（启动斜坡）、motor.c 的 `VELOCITY_KP/KI`。

**注意**：`pid_calc`（motor.c）在 `target == 0` 时会**清零**积分而非冻结。
满矫正需要内轮真的抱死，残留积分（上限 +40，直行才 80）会让本该停住的内轮往前冲。

## Project Structure

- `main.c` — 主程序：状态机（STOP/RUN）、巡线算法、10ms ISR 速度环、丢线保护
- `Hardware/` — 驱动模块：
  - `common.h` — 通用宏（CLAMP）
  - `system/board.c/.h` — 延时、UART0 收发、printf 重定向（调试用）
  - `motor/motor.c/.h` — 四路电机 PI 速度环（`Velocity_A~D`）、PWM 输出（`Set_PWM`）
  - `motor/encoder.c/.h` — 四路正交编码器计数（GPIO 中断）
  - `sensor/grayscale_sensor.c/.h` — 8 路灰度传感器（74HC4051 + ADC，动态自适应归一化）
  - `ui/key.c/.h` — 按键单击检测（50ms 消抖状态机）
  - `ui/led.c/.h` — LED 控制（备用，未接入）
  - `display/oled_hardware_i2c.c/.h`、`oledfont.h` — SSD1306 OLED 驱动
- `base.syscfg` — TI SysConfig 引脚/外设配置（ADC、GPIO、PWM、UART、TIMER、I2C、SYSTICK）
- `targetConfigs/MSPM0G3507.ccxml` — XDS110 调试器目标配置
- `Debug/ti_msp_dl_config.c/.h` — SysConfig 自动生成的 DriverLib 配置（**勿手改**）

## Hardware Configuration

- **MCU**: MSPM0G3507 (LQFP-64)，32MHz
- **Debug**: XDS110 via SWD（SWCLK=PA20, SWDIO=PA19）
- **UART**: UART0（TX=PA10, RX=PA11, 115200）
- **PWM**: TIMA0 中心对齐，周期 1000（=100% 占空比），4 通道 PA8/PB9/PB20/PB13
- **ADC**: ADC0 通道 0，PA27，repeat 模式软件触发
- **Timer**: TIMA1 10ms 周期中断（BUSCLK/8/200）
- **I2C**: I2C0 400kHz（SDA=PA28, SCL=PA1），OLED 0x3C
- **灰度传感器**: 74HC4051 多路开关（AD0=PA24, AD1=PB24, AD2=PA30），OUT=PA27
- **编码器**: 4 路正交编码器，GPIOA/B 上升沿中断（A: PA15/16/17/22，B: PB6/7/8/15）
- **电机驱动**: 4×（IN1/IN2 + PWM），双高电平 = 制动
- **按键**: 3 颗，上拉输入（PB21/PB23/PA23）
- **LED**: PA0，低电平点亮

## Build

- **IDE**: TI Code Composer Studio (CCS)
- **Toolchain**: TI Clang (ticlang)
- **SDK**: MSPM0 SDK 2.10.00.04
- **SysConfig**: 1.27.0
- 构建产物在 `Debug/`（已 gitignore），用 CCS 构建后烧录

## Commits

- 提交信息写清楚**为什么**改
- 构建产物、IDE 文件、生成代码不入库（见 `.gitignore`）
