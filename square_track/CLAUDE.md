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

MSPM0G3507 (ARM Cortex-M0+) **正方形巡线**工程（边长 1m，四个直角弯），从 `base3.6`（H 题环形赛道）复制而来，
驱动层完全相同。8 路灰度传感器（74HC4051 多路复用 + ADC）巡线，四路直流电机带编码器闭环 PI 速度控制，OLED 显示状态。

- 工程名/文件夹必须是 ASCII：路径里有中文时 SysConfig CLI 会把路径编码弄乱，直接找不到 `base.syscfg`

## 架构（重要）

**速度环在 10ms 定时器中断里执行**（固定控制周期），主循环只做非实时任务：

- `TIMER_0_INST_IRQHandler`（10ms）：读编码器计数 → 累加里程 `odo_cnt` → 四路 PI 速度环 → PWM 输出 → 启动斜坡；
  停车后输出制动脉冲（`ctrl_brake`）
- 主循环（轮询 `flag_10ms`，不用 `__WFI()`）：灰度扫描（≈1.7ms）→ `Square_Step()`（直边巡线 / 直角弯）→
  通过 volatile 变量发布目标（`ctrl_target_l/r`、`ctrl_enable`）→ 遥测 → OLED 画一个字符 → 按键
  - **主循环一拍必须远小于 10ms**，超了会静默丢拍（这一拍不读灰度）
  - OLED 驱动每个字节单独一次 I2C（~80us），8x16 的字一个要 ~2ms，所以 `OLED_Refresh_Step`
    每拍只画一个字符，18 个字符轮一遍 180ms。**别在主循环里一次画整屏或调 `OLED_Clear`**（整屏 ~85ms）
- OLED：第一行状态 `[STOP]` / `[RUN ]` / `[LOOP]`，中间计时 `SS.HHs`，第三行已转过的弯 `CNR nn`

## 直边巡线（main.c → Follow_Line）

1. **位置**：传感器下标 ×100 − 350，范围 ±350（S3/S4 中间 = 0）。
   `position > 0` = 线偏左 → 左转；`digital[0]` 是小车最右侧那一路
   - `TRACK_ANALOG_POS = 1`：压线那几路和紧邻两路按电平（`level[]`，本帧归一化 0~100）
     减 `POS_LEVEL_FLOOR` 加权，线在两路之间时连续插值；`= 0` 只看哪几路压线，一格 50 地跳
2. **D 项**：差分 `position`（`/25` 换算），两点平均（`filtered_d`），再一阶平滑（`smooth_pos`，3/4 旧 + 1/4 新）
   - **不能差分权重和**：权重非线性，压线路数一变就跳，线单调移动时 D 会反向推
3. **转向强度**（千分制）：`lin = |position| / POS_DIVISOR`，`turn = TURN_LINEAR% × lin + (100 − TURN_LINEAR)% × lin²`。
   `POS_DIVISOR = 250`（别低于 150），`TURN_LINEAR = 30`
   - **不能用纯平方（`TURN_LINEAR = 0`）**：线附近增益几乎为 0，偏半路才 4%，再被整数速度取整成 0，
     线两侧各 ~1cm 内完全不纠偏，车贴着死区边沿走、慢慢偏出直线（仿真直边偏差 ~1cm → 加线性后 ~0.1cm）
4. **内外轮**：外轮 = tgt + (max − tgt) × turn‰，内轮 = tgt − (tgt − min) × turn‰（`SQ_*_SPEED` = 6/18/2）
   - **速度目标是细分单位**：`SPEED_SCALE`（motor.h）= 10，目标 = 0.1 计数/10ms，motor.c 的 PI 按细分算误差和积分。
     宏里的速度都按计数写，用的地方乘 `SPEED_SCALE`；新加速度常量别忘了乘
   - motor.c `PID_MAX_OUT = 300`：目标超过 ~15 就饱和
5. **宽线**（`black_cnt >= 4`，即拐角那段横线）：回正直行越过去
6. **速度斜坡**：每 10ms 左右轮目标最多变 `SPEED_RAMP`；每条出口都必须经过 `Ramp_Targets`
- 依赖 `grayscale_sensor.c`：8 路 max−min < `GRAY_CONTRAST_MIN` 时按绝对电平区分"全在地面"（全 0）和"全压线"（全 1）；
  ADC 超时返回 `0xFFFF` 时沿用上一次有效值

## 直角弯与回出发点（main.c → Square_Step）

- `Scan_Line` 每帧记 `line_side`（线最后在哪一侧，和 position 同号）：**只有一边的边缘两路（S0/S1 或 S6/S7）
  压线才更新**。拐角横线一定伸到那一边的边缘；两边都压是十字/横线，不更新
- 三段：`SEG_FOLLOW`（直边）→ `SEG_TURN`（原地转）→ `SEG_SETTLE`（低速对正 `SETTLE_MS`）→ `SEG_FOLLOW`
- 直边上连续 `CORNER_LOST_N` 帧全白、且 `line_side` 已知 → `Corner_Begin` 进入 `SEG_TURN`：
  外侧轮 `CORNER_OUTER`（5，比巡航慢，转太快惯性大会转过头）、内侧轮 `CORNER_INNER`；
  转够 `CORNER_MIN_MS` 后任一路看到新边就降到 `CORNER_SLOW`；S3/S4 压线 → `Corner_End` 进入对正
  - **`CORNER_INNER = 0`（绕内侧轮转）是有意的**：灰度在车头前面，绕车身中心原地转（内侧轮反转）
    一开转就扫回刚越过的横线，停在 20° 左右；绕内侧轮转先往外甩，在 60°~100° 扫到新边，剩下的由巡线补
  - `Corner_End` 要 `Reset_Track_State()`（否则 D 项把转弯前后的位置差当成猛偏），并把 `smooth_l/r`
    都拉到内侧轮速度（车立刻停止旋转，不会转过头）
- **对正（`SEG_SETTLE`）**：四驱原地转时刹住的一侧被拖着往后滑，转弯中心比理想靠里，S3/S4 可能在车头只转了
  ~40° 时就扫到新边（仿真复现过）。对正用 `SETTLE_*_SPEED`（3/10/-3）巡线：低速、内侧轮可倒转、斜坡用 `WIDE_RAMP_STEP`，
  车头差得多时接近原地转；横跨 ≥4 路（斜压）时接着原地转
  - **对正期间丢线 = 同一个弯没转好**：`Corner_Begin(false)`，朝 `line_side` 接着转，**不计数、不记里程**。
    以前算成新弯，一个弯数成两三个（OLED 的 CNR 比实际多），键1 数够 4 个就回出发点停车 → "跑一会就停"
  - 转完第一帧用实测位置给 D 项和平滑起步（`track_primed`），不从 0 慢慢爬
- **不自动停车**（用户要求）：没有转弯超时、丢线超时；`SQUARE_LAPS = 0`（默认）键1/键2 都一直跑，只有键3 停
- **回出发点（不用标定里程）**：A = 出发 → 第 1 个弯开始转，B = 第 1 个弯转完 → 第 2 个弯开始转（一整条边）。
  `SQUARE_LAPS > 0` 时，第 `4 × SQUARE_LAPS` 个弯转完后再走 B − A 就停在出发时的位置。出发点要在直边上、离前面的拐角至少一车长
- 键1 `STATE_RUN`（`SQUARE_LAPS > 0` 时跑够圈数回出发点停），键2 `STATE_LOOP`（一直跑），再按一次停；键3 停车
- `Stop_Car`：先关速度环再挂 `ctrl_brake`；ISR 只在车还往前走时倒转（最多 80ms），轮速归零立刻双高制动
- 仿真（差速运动学 + 刹住一侧后滑 0~0.7 + 电机 τ 80~250ms + 灰度延迟 0~3 拍，顺/逆时针）360/360 次转完 4 个弯
  回到出发点；真车以实测为准。COM11（XDS110 背通道）在这块板上收不到遥测，要看遥测得另接串口

## 调参指南

| 现象 | 调整 |
|---|---|
| 想更快/更慢 | `SQ_TARGET_SPEED`（×19 ≈ PWM；超过 ~15 会撞 `PID_MAX_OUT`） |
| 直边慢慢偏出线 | `TURN_LINEAR` 30→50（线附近纠偏更硬） |
| 直边蛇形摆动 | `TURN_LINEAR` 30→15，或 `POS_DIVISOR` 调大、`TRACK_D_GAIN` 减小；插值后抖就 `POS_LEVEL_FLOOR` 20→30 |
| 一个弯转两次 / 来回转 | `CORNER_OUTER` 5→4、`CORNER_SLOW` 4→3（转慢点少冲过头）；`SETTLE_MS` 600→900 |
| 转完对正时还是丢线 | `SETTLE_SPEED` 3→2，`SETTLE_MIN_SPEED` -3→-5（更接近原地转） |
| 转完弯转过头、扫到别的线 | `CORNER_INNER` 0→1（弧线大一点），或 `CORNER_MIN_MS` 调大 |
| 原地转时轮子不动/卡住 | `CORNER_SLOW` 太小进了电机死区，调回 4~5 |
| 回出发点偏前/偏后 | 出发点离拐角太近（< 一车长）会测不准 A；或在 `Corner_End` 里给 `home_dist` 加减一点 |
| 整体偏一侧（机械） | `LEFT_TRIM` 100→103 等 |
| 线极性反了 | `SENSOR_INVERT`（grayscale_sensor.c）0↔1 |

其他可调：`SPEED_RAMP`、`STARTUP_RAMP_*`（启动斜坡）、motor.c 的 `VELOCITY_KP/KI`。

遥测每行的目标/实际速度都是细分单位（×10），两者可直接比。

**注意**：`pid_calc`（motor.c）在 `target == 0` 时会**清零**积分而非冻结，残留积分会把本该停住的轮子顶出去
（转弯时内侧轮目标就是 0）。

## Project Structure

- `main.c` — 主程序：状态机（STOP/RUN/LOOP）、直边巡线、直角弯状态机、回出发点、10ms ISR 速度环、OLED 分摊刷新
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
