#include "ti_msp_dl_config.h"
#include "motor/motor.h"
#include "motor/encoder.h"
#include "sensor/grayscale_sensor.h"
#include <stdint.h>
#include <stdbool.h>
#include "display/oled_hardware_i2c.h"
#include "ui/key.h"
#include "system/board.h"

/* 正方形巡线（边长 1m，四个直角弯）。直边沿用 base3.6 的巡线算法；
 * 直角弯：越过拐角横线、全白 → 绕内侧轮原地转，直到中间两路重新压线 → 低速对正 → 回到直边。
 * 默认一直跑不自动停；SQUARE_LAPS > 0 时键1 跑够圈数回到出发点停 */

volatile int32_t encoderA_cnt = 0, encoderB_cnt = 0, encoderC_cnt = 0, encoderD_cnt = 0;

#define SENSOR_NUM        8
#define TRACK_D_GAIN      12

/* 250 = 阵列边缘（|position| 最大才 350）。改回参考版的 100 会让转向
 * 饱和成三层继电器，实测 1.9Hz 极限环 —— 想更跟线可以降到 200，别低于 150 */
#define POS_DIVISOR       250

/* 转向强度 = 线性 TURN_LINEAR% + 平方 (100 − TURN_LINEAR)%，两端（0 和满舵）不变。
 * 纯平方在线附近增益几乎为 0：偏半路只有 4%，再被整数速度一取整就是 0，
 * 线两侧各 ~1cm 内根本不纠偏，车贴着死区边沿走、慢慢偏出去。0 = 原来的纯平方 */
#define TURN_LINEAR       30

// 1 = 位置按灰度电平插值（连续）；0 = 只看哪几路压线（一格 50 地跳）
#define TRACK_ANALOG_POS  1
#define POS_LEVEL_FLOOR   20    // 电平低于它的不参与插值（地面噪声）
#define LEFT_TRIM         100    // 左轮机械补偿（%）

// 下面所有速度都按"计数/10ms"写，用的地方再乘 SPEED_SCALE（motor.h）变成细分单位
#define SPEED_RAMP         1     // 每 10ms 速度目标最大变化量
#define WIDE_RAMP_STEP      4    // 越过拐角横线、进出转弯用的斜率，比正常转向陡

/* 直边速度（目标/最大/最小），沿用 base3.6 慢模式。
 * 注意 motor.c 的 PID_MAX_OUT = 300：目标 ×19 超过 300 就饱和 */
#define SQ_TARGET_SPEED     6
#define SQ_MAX_SPEED        18
#define SQ_MIN_SPEED        2

/* 直角弯：
 * 1. 直边上压到拐角那段横线（≥4 路）→ 回正直行越过去
 * 2. 连续 CORNER_LOST_N 帧全白 → 朝线最后出现的一侧转：外侧轮 CORNER_OUTER，内侧轮 CORNER_INNER；
 *    转够 CORNER_MIN_MS 后只要有一路看到新边就降到 CORNER_SLOW
 * 3. 中间两路（S3/S4）压到线 → 低速对正 SETTLE_MS → 回到直边巡线
 * CORNER_INNER = 0 是绕内侧轮转。灰度在车头前面，绕车身中心原地转（内侧轮反转）的话
 * 一开转就扫回刚越过的那段横线；绕内侧轮转会先往外甩，再扫到新边。
 * 外侧轮比巡航慢：转得快惯性大，停转后车还会多转一截，线从另一边跑出去 → 又转一次 */
#define CORNER_LOST_N        2
#define CORNER_OUTER         5
#define CORNER_SLOW          4
#define CORNER_INNER         0
#define CORNER_MIN_MS        150

/* 对正：四驱原地转时刹住的一侧会被拖着往后滑，转弯中心比理想靠里，中间两路可能在车头
 * 才转了 40° 时就扫到新边。对正期间巡航降到 SETTLE_SPEED、内侧轮允许倒转
 * （SETTLE_MIN_SPEED < 0），车头差得多时接近原地转、几乎不往前走，线不会从边上跑出去。
 * ★对正期间丢线 = 这个弯没转好（不够或转过头），朝线最后出现的一侧接着转，不算新弯。
 * 原来算成新弯：一个弯数成两三个，键1 数够 4 个就回出发点停车，看起来就是"跑一会就停" */
#define SETTLE_SPEED         3
#define SETTLE_MAX_SPEED     10
#define SETTLE_MIN_SPEED     (-3)
#define SETTLE_MS            600

/* 0 = 键1、键2 都一直跑，不自动停（默认）。
 * n > 0 = 键1 跑 n 圈（每圈 4 个弯）后回到出发点停车。回出发点不用标定里程：
 * A = 出发 → 第 1 个弯，B = 第 1 个弯转完 → 第 2 个弯（一整条边），最后一个弯转完再走 B − A。
 * 出发点要在直边上，离前面的拐角至少一车长 */
#define SQUARE_LAPS          0

#define STARTUP_RAMP_MAX    100
#define STARTUP_RAMP_STEP   4     // 250ms 到满速

#define TELEMETRY_ENABLE     1
#define TELEMETRY_RAW        1
#define TELEMETRY_EVERY_N    2     // 每 N 个 10ms 周期发一行（2 = 50Hz）

// 实测校准，勿改
#define ENCODER_AC_INVERT   1
#define ENCODER_BD_INVERT   0

typedef enum {
    STATE_STOP = 0,
    STATE_RUN,       // 键1：SQUARE_LAPS > 0 时跑够圈数回出发点停
    STATE_LOOP       // 键2：一直跑
} Car_State_t;

typedef enum {
    SEG_FOLLOW = 0,  // 直边巡线
    SEG_TURN,        // 直角弯原地转
    SEG_SETTLE       // 转完低速对正
} Seg_t;

static GraySensor_Handle gray_sensor;
static Car_State_t car_state = STATE_STOP;

static int32_t track_last_pos = 0;
static int32_t filtered_d = 0;
static int32_t smooth_pos = 0;
static bool    track_primed = false;   // false = 下一帧用实测位置给 D 项和平滑起步
static int16_t smooth_l = 0, smooth_r = 0;

static uint32_t mode_entry_ms = 0;
static uint32_t stopwatch_final_ms = 0;

static Seg_t    seg = SEG_FOLLOW;
static uint32_t seg_entry_ms = 0;
static int8_t   line_side = 0;      // 线最后出现在哪一侧，和 position 同号：+1 左 / -1 右 / 0 不知道
static int8_t   turn_dir = 0;
static bool     turn_counted = true;   // 这次转弯算不算新弯（对正时丢线接着转的不算）
static uint8_t  lost_cnt = 0;
static uint8_t  corner_cnt = 0;
static int32_t  turn_end_odo = 0;   // 上一个弯转完时的里程
static int32_t  dist_a = 0;         // 出发 → 第 1 个弯
static int32_t  dist_b = 0;         // 第 1 个弯转完 → 第 2 个弯（一整条边）
static bool     homing = false;     // 圈数够了，正在走回出发点
static int32_t  home_dist = 0;

static volatile int16_t ctrl_target_l = 0;
static volatile int16_t ctrl_target_r = 0;
static volatile uint8_t ctrl_enable   = 0;
static volatile uint16_t startup_ramp = 0;

// 停车制动脉冲剩余拍数，由 ISR 递减并输出（主循环只负责置一次）
static volatile uint8_t ctrl_brake = 0;

// 里程计 = 四轮编码器计数之和，ISR 在行驶/制动时累加，Enter_Mode 清零
static volatile int32_t odo_cnt = 0;

static volatile int16_t wheel_target_abcd[4] = {0};
static uint16_t tele_div = 0;

volatile uint32_t system_ms = 0;

volatile uint8_t  flag_10ms = 0;

static void Square_Step(int16_t *left_spd, int16_t *right_spd);
static uint8_t Scan_Line(int32_t *sum_pos);
static void Follow_Line(int16_t *left_spd, int16_t *right_spd, uint8_t black_cnt, int32_t sum_pos, bool settle);
static void Turn_Output(int16_t *left_spd, int16_t *right_spd, int16_t outer);
static void Corner_Begin(bool counted);
static void Corner_End(void);
static void Ramp_Targets(int16_t *left_spd, int16_t *right_spd, int16_t step);
static void Reset_Track_State(void);
static int16_t Trim_Left(int16_t spd);
static void Apply_Motor_Speeds(int16_t left_spd, int16_t right_spd, uint16_t ramp, uint16_t ramp_max);
static void Reset_PWM_Filter(void);
static uint32_t Get_Current_Elapsed(void);
static void Stop_Car(void);
static void Enter_Mode(Car_State_t st);
static void OLED_Refresh_Step(void);
static char *Tele_Append_Int(char *p, int32_t v);
static void Send_Wheel_Telemetry(void);

int main(void)
{
    SYSCFG_DL_init();

    NVIC_ClearPendingIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_ClearPendingIRQ(GPIOB_INT_IRQn);
    NVIC_ClearPendingIRQ(TIMER_0_INST_INT_IRQN);

    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    NVIC_EnableIRQ(GPIOB_INT_IRQn);
    NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);   // 必须早于 OLED_Init，I2C 超时保护依赖它
    NVIC_EnableIRQ(UART_0_INST_INT_IRQN);    // SysConfig 只使能外设中断，NVIC 要补

    GraySensor_Init(&gray_sensor);
    OLED_Init();
    OLED_Clear();

    while (1) {
        /* 不用 __WFI()：先睡再判标志的写法在某一拍超时时会静默丢拍 */
        if (!flag_10ms) continue;
        flag_10ms = 0;

        int16_t left_spd = 0, right_spd = 0;

        switch (car_state) {

            case STATE_STOP: {
                ctrl_enable = 0;
                ctrl_target_l = ctrl_target_r = 0;
                Reset_Motor_Integrals();
                Reset_PWM_Filter();
#if TELEMETRY_ENABLE
                wheel_target_abcd[0] = wheel_target_abcd[1] = 0;
                wheel_target_abcd[2] = wheel_target_abcd[3] = 0;
#endif
                GraySensor_Update(&gray_sensor);
                break;
            }

            case STATE_RUN:
            case STATE_LOOP: {
                GraySensor_Update(&gray_sensor);
                Square_Step(&left_spd, &right_spd);

                if (car_state == STATE_STOP)   // 回到出发点 / 找不到线，Square_Step 里已经停车
                    break;

                ctrl_target_l = left_spd;
                ctrl_target_r = right_spd;
                ctrl_enable = 1;

#if TELEMETRY_ENABLE
                if (++tele_div >= TELEMETRY_EVERY_N) {
                    tele_div = 0;
                    Send_Wheel_Telemetry();
                }
#endif
                break;
            }
        }

        OLED_Refresh_Step();

        /* 键1 跑圈（到圈数回出发点停）/ 键2 一直跑 / 键3 停车。键1、键2 是开关：
         * 再按一次停，在对方模式下按则切过去 */
        uint8_t run_clicked = 0;

        if (key1_click()) {
            run_clicked = 1;
            if (car_state == STATE_RUN) Stop_Car();
            else                        Enter_Mode(STATE_RUN);
        }
        if (key2_click()) {
            run_clicked = 1;
            if (car_state == STATE_LOOP) Stop_Car();
            else                         Enter_Mode(STATE_LOOP);
        }
        if (key3_click() && !run_clicked && car_state != STATE_STOP) {
            Stop_Car();
        }
    }
}

static uint32_t Get_Current_Elapsed(void)
{
    return system_ms - mode_entry_ms;
}

static void Stop_Car(void)
{
    stopwatch_final_ms = Get_Current_Elapsed();
    car_state = STATE_STOP;

    /* ★必须先关速度环再挂制动脉冲：ISR 的分支是 if(ctrl_enable) ... else if(ctrl_brake)，
     * 只设 ctrl_brake 的话这一拍还会白跑一次全速 */
    ctrl_enable = 0;
    ctrl_target_l = ctrl_target_r = 0;
    ctrl_brake = 8;      // 最多 8 拍 = 80ms 反转，轮子一停就提前结束（见 ISR）
}

// 这里不碰 OLED：OLED_Clear 要 ~85ms，秒表已经在走、车却还没动
static void Enter_Mode(Car_State_t st)
{
    car_state = st;
    mode_entry_ms = system_ms;
    stopwatch_final_ms = 0;
    Reset_Motor_Integrals();
    Reset_PWM_Filter();
    Reset_Track_State();
    smooth_l = smooth_r = 0;
    startup_ramp = 0;
    odo_cnt = 0;

    seg = SEG_FOLLOW;
    seg_entry_ms = system_ms;
    line_side = 0;
    turn_dir = 0;
    turn_counted = true;
    lost_cnt = 0;
    corner_cnt = 0;
    turn_end_odo = 0;
    dist_a = dist_b = 0;
    homing = false;
    home_dist = 0;
}

// 每拍一次：直边巡线 → 直角弯原地转 → 低速对正 → 直边巡线
static void Square_Step(int16_t *left_spd, int16_t *right_spd)
{
    int32_t  sum_pos;
    uint8_t  black_cnt = Scan_Line(&sum_pos);
    uint32_t seg_ms = system_ms - seg_entry_ms;

    if (seg == SEG_TURN) {
        if (seg_ms >= CORNER_MIN_MS && (gray_sensor.digital[3] || gray_sensor.digital[4])) {
            Corner_End();   // → 对正，本帧接着往下走
        } else {
            Turn_Output(left_spd, right_spd,
                        (seg_ms >= CORNER_MIN_MS && black_cnt > 0) ? CORNER_SLOW : CORNER_OUTER);
            return;
        }
    }

    if (black_cnt == 0) {
        if (lost_cnt < 255)
            lost_cnt++;
        if (lost_cnt >= CORNER_LOST_N && line_side != 0) {
            bool redo = (seg == SEG_SETTLE);   // 对正时丢线：这个弯没转好，接着转，不算新弯
            Corner_Begin(!redo);
            Turn_Output(left_spd, right_spd, redo ? CORNER_SLOW : CORNER_OUTER);
            return;
        }
        // 还没确认是拐角（或还不知道线在哪侧）：先保持直行
        *left_spd  = ((seg == SEG_SETTLE) ? SETTLE_SPEED : SQ_TARGET_SPEED) * SPEED_SCALE;
        *right_spd = *left_spd;
        Ramp_Targets(left_spd, right_spd, SPEED_RAMP * SPEED_SCALE);
        return;
    }

    lost_cnt = 0;

    if (seg == SEG_SETTLE) {
        if (black_cnt >= 4)   // 斜着横跨好几路：车头还差得多，接着原地转
            Turn_Output(left_spd, right_spd, CORNER_SLOW);
        else
            Follow_Line(left_spd, right_spd, black_cnt, sum_pos, true);

        if (seg_ms >= SETTLE_MS) {
            seg = SEG_FOLLOW;
            seg_entry_ms = system_ms;
        }
    } else {
        Follow_Line(left_spd, right_spd, black_cnt, sum_pos, false);
    }

    if (homing && odo_cnt - turn_end_odo >= home_dist)
        Stop_Car();
}

/* 数压线路数、累加位置；顺带记线最后在哪一侧。只有一边的边缘两路压线才更新：
 * 拐角横线一定伸到那一边的边缘，两边都压是十字/横线，不更新 */
static uint8_t Scan_Line(int32_t *sum_pos)
{
    uint8_t black_cnt = 0;

    *sum_pos = 0;
    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        if (gray_sensor.digital[i]) {
            black_cnt++;
            *sum_pos += (int32_t)i * 100;
        }
    }

    // digital[0] 是小车最右侧那一路
    bool right_edge = gray_sensor.digital[0] || gray_sensor.digital[1];
    bool left_edge  = gray_sensor.digital[SENSOR_NUM - 1] || gray_sensor.digital[SENSOR_NUM - 2];
    if (right_edge != left_edge)
        line_side = left_edge ? 1 : -1;

    return black_cnt;
}

/* line_side / turn_dir 和 position 同号：> 0 = 线在左 → 左转 → 左轮是内侧。
 * outer 是外侧轮速度（计数/10ms） */
static void Turn_Output(int16_t *left_spd, int16_t *right_spd, int16_t outer)
{
    int16_t out_spd = outer * SPEED_SCALE;
    int16_t in_spd  = CORNER_INNER * SPEED_SCALE;

    *left_spd  = (turn_dir > 0) ? in_spd : out_spd;
    *right_spd = (turn_dir > 0) ? out_spd : in_spd;
    Ramp_Targets(left_spd, right_spd, WIDE_RAMP_STEP * SPEED_SCALE);
}

// counted = false：对正时丢线接着转，是同一个弯，不计数也不记里程
static void Corner_Begin(bool counted)
{
    if (counted) {
        if (corner_cnt == 0)
            dist_a = odo_cnt;               // 发车时里程清零，这里就是出发 → 第 1 个弯
        else if (corner_cnt == 1)
            dist_b = odo_cnt - turn_end_odo;
    }

    turn_counted = counted;
    turn_dir = line_side;
    seg = SEG_TURN;
    seg_entry_ms = system_ms;
    lost_cnt = 0;
}

static void Corner_End(void)
{
    if (turn_counted)
        corner_cnt++;
    turn_end_odo = odo_cnt;
    seg = SEG_SETTLE;
    seg_entry_ms = system_ms;

    // 转弯前的位置作废，不然 D 项会把转弯前后的位置差当成一次猛偏
    Reset_Track_State();
    // 两侧斜坡起点都拉到内侧轮的速度：车立刻停止旋转、不会转过头，再一起加速
    smooth_l = smooth_r = (smooth_l < smooth_r) ? smooth_l : smooth_r;

    if (turn_counted && car_state == STATE_RUN && SQUARE_LAPS > 0 && corner_cnt >= 4 * SQUARE_LAPS) {
        homing = true;
        home_dist = (dist_b > dist_a) ? dist_b - dist_a : 0;
    }
}

/* ★每条出口都必须经过它：它同时更新 smooth_l/r，漏掉的话限速器会拿过期的起点做基准，
 * 目标被反向弹一次 */
static void Ramp_Targets(int16_t *left_spd, int16_t *right_spd, int16_t step)
{
    int16_t *spd[2]    = {left_spd, right_spd};
    int16_t *smooth[2] = {&smooth_l, &smooth_r};

    for (int j = 0; j < 2; j++) {
        if (*spd[j] > *smooth[j] + step)
            *spd[j] = *smooth[j] + step;
        else if (*spd[j] < *smooth[j] - step)
            *spd[j] = *smooth[j] - step;
        *smooth[j] = *spd[j];
    }
}

/* ★只在发车和转完弯时调用：丢线/宽线期间每拍清零的话，
 * 重新压线时 D 项会把整个 position 当成差分，车猛甩一下。
 * 这里也不清 smooth_l/r，那是速度斜坡的状态，由 Ramp_Targets 管 */
static void Reset_Track_State(void)
{
    track_last_pos = 0;
    filtered_d = 0;
    smooth_pos = 0;
    track_primed = false;
}

static int16_t Trim_Left(int16_t spd)
{
    return (spd > 0) ? (int16_t)(spd * LEFT_TRIM / 100) : spd;
}

// PWM 输出一阶低通：τ ≈ 2^PWM_LPF_SHIFT × 10ms。调大会加相位滞后，反而喂大极限环
#define PWM_LPF_SHIFT   1

static int32_t pwm_lpf[4] = {0};

static void Reset_PWM_Filter(void)
{
    pwm_lpf[0] = pwm_lpf[1] = pwm_lpf[2] = pwm_lpf[3] = 0;
}

static int32_t Pwm_LPF(uint8_t w, int32_t x)
{
    pwm_lpf[w] += (x - pwm_lpf[w]) / (1 << PWM_LPF_SHIFT);
    return pwm_lpf[w];
}

static void Apply_Motor_Speeds(int16_t left_spd, int16_t right_spd, uint16_t ramp, uint16_t ramp_max)
{
    left_spd = Trim_Left(left_spd);

    wheel_target_abcd[0] = right_spd;   /* A（右轮）*/
    wheel_target_abcd[1] = left_spd;    /* B（左轮）*/
    wheel_target_abcd[2] = right_spd;   /* C（右轮）*/
    wheel_target_abcd[3] = left_spd;    /* D（左轮）*/

    Motor_Set_Integral_Enable(ramp >= ramp_max);

    // 实测：A/C 装在右轮、B/D 在左轮，故此处交换左右目标速度
    int pwma = Velocity_A(right_spd, encoderA_cnt);
    int pwmb = Velocity_B(left_spd, encoderB_cnt);
    int pwmc = Velocity_C(right_spd, encoderC_cnt);
    int pwmd = Velocity_D(left_spd, encoderD_cnt);

    if (ramp >= ramp_max) ramp = ramp_max;

    pwma = pwma * (int)ramp / (int)ramp_max;
    pwmb = pwmb * (int)ramp / (int)ramp_max;
    pwmc = pwmc * (int)ramp / (int)ramp_max;
    pwmd = pwmd * (int)ramp / (int)ramp_max;

    Set_PWM((int16_t)Pwm_LPF(0, pwma), (int16_t)Pwm_LPF(1, pwmb),
            (int16_t)Pwm_LPF(2, pwmc), (int16_t)Pwm_LPF(3, pwmd));
}

void TIMER_0_INST_IRQHandler(void)
{
    if (DL_TimerA_getPendingInterrupt(TIMER_0_INST) == DL_TIMER_IIDX_ZERO)
    {
        system_ms += 10;

        NVIC_DisableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
        NVIC_DisableIRQ(GPIOB_INT_IRQn);

        encoderA_cnt = ENCODER_AC_INVERT ? -Get_Encoder_countA : Get_Encoder_countA;
        encoderB_cnt = ENCODER_BD_INVERT ? -Get_Encoder_countB : Get_Encoder_countB;
        encoderC_cnt = ENCODER_AC_INVERT ? -Get_Encoder_countC : Get_Encoder_countC;
        encoderD_cnt = ENCODER_BD_INVERT ? -Get_Encoder_countD : Get_Encoder_countD;

        Get_Encoder_countA = 0;
        Get_Encoder_countB = 0;
        Get_Encoder_countC = 0;
        Get_Encoder_countD = 0;

        NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
        NVIC_EnableIRQ(GPIOB_INT_IRQn);

        int32_t moved = encoderA_cnt + encoderB_cnt + encoderC_cnt + encoderD_cnt;
        if (ctrl_enable || ctrl_brake)
            odo_cnt += moved;

        if (ctrl_enable) {
            if (startup_ramp < STARTUP_RAMP_MAX) {
                startup_ramp += STARTUP_RAMP_STEP;
                if (startup_ramp > STARTUP_RAMP_MAX)
                    startup_ramp = STARTUP_RAMP_MAX;
            }
            Apply_Motor_Speeds(ctrl_target_l, ctrl_target_r, startup_ramp, STARTUP_RAMP_MAX);
        } else if (ctrl_brake) {
            /* 固定倒转 80ms 在低速时会把车往回拉；只在车还往前走时反转，
             * 速度归零立刻转双高制动 */
            if (moved <= 0) {
                ctrl_brake = 0;
                Set_PWM(0, 0, 0, 0);
            } else {
                ctrl_brake--;
                Set_PWM(-400, -400, -400, -400);   // 真的倒转四轮，不是刹车
            }
        } else {
            Set_PWM(0, 0, 0, 0);   // PWM=0 即双高制动
        }

        flag_10ms = 1;
    }
}

/* 直边巡线，black_cnt ≥ 1 时调用（全白由 Square_Step 处理）。输出是细分单位。
 * settle = 转完弯对正：低速、内侧轮可倒转、斜坡陡，车头差得多时接近原地转 */
static void Follow_Line(int16_t *left_spd, int16_t *right_spd, uint8_t black_cnt, int32_t sum_pos, bool settle)
{
    const int32_t tgt   = (settle ? SETTLE_SPEED     : SQ_TARGET_SPEED) * SPEED_SCALE;
    const int32_t max_s = (settle ? SETTLE_MAX_SPEED : SQ_MAX_SPEED)    * SPEED_SCALE;
    const int32_t min_s = (settle ? SETTLE_MIN_SPEED : SQ_MIN_SPEED)    * SPEED_SCALE;
    const int16_t step  = (settle ? WIDE_RAMP_STEP   : SPEED_RAMP)      * SPEED_SCALE;

    // 拐角横线：位置算不出来，回正直行越过去，越过后全白再转弯
    if (black_cnt >= 4) {
        *left_spd  = (int16_t)tgt;
        *right_spd = (int16_t)tgt;
        Ramp_Targets(left_spd, right_spd, WIDE_RAMP_STEP * SPEED_SCALE);
        return;
    }

    int32_t position = sum_pos / black_cnt - 350;

#if TRACK_ANALOG_POS
    /* 只取压线那几路和紧挨着的两路按电平加权，远处地面上的噪声不参与。
     * 压线的那路电平 ≥ 45（二值化滞回下限），所以 wsum 不会是 0 */
    int32_t wsum = 0, psum = 0;
    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        bool near = gray_sensor.digital[i]
                 || (i > 0 && gray_sensor.digital[i - 1])
                 || (i + 1 < SENSOR_NUM && gray_sensor.digital[i + 1]);
        int32_t w = (int32_t)gray_sensor.level[i] - POS_LEVEL_FLOOR;

        if (near && w > 0) {
            wsum += w;
            psum += w * (int32_t)i * 100;
        }
    }
    if (wsum > 0)
        position = psum / wsum - 350;
#endif

    // 发车/转完弯后的第一帧用实测位置起步：平滑不用从 0 慢慢爬（滞后），D 项也不出冲击
    if (!track_primed) {
        track_last_pos = position;
        smooth_pos = position;
        track_primed = true;
    }

    /* ★D 项差分 position，不能差分权重和：权重非线性，压线路数一变就跳。
     * /25 让 TRACK_D_GAIN 的量级和原来差分权重和时相当 */
    int32_t d_pos = position - track_last_pos;
    track_last_pos = position;
    filtered_d = (filtered_d + d_pos) / 2;
    position += filtered_d * TRACK_D_GAIN / 25;

    smooth_pos = (smooth_pos * 3 + position) / 4;
    position = smooth_pos;

    int32_t abs_pos = (position >= 0) ? position : -position;
    if (abs_pos > POS_DIVISOR) abs_pos = POS_DIVISOR;

    // 转向强度 0~1000（千分制，配合细分速度，小偏差也能出小差速）
    int32_t lin  = abs_pos * 1000 / POS_DIVISOR;
    int32_t turn = (lin * TURN_LINEAR + (lin * lin / 1000) * (100 - TURN_LINEAR)) / 100;

    int32_t outer = tgt + (max_s - tgt) * turn / 1000;
    int32_t inner = tgt - (tgt - min_s) * turn / 1000;

    // position > 0 = 线偏左 → 左轮慢 → 左转
    if (position > 0) {
        *left_spd  = (int16_t)inner;
        *right_spd = (int16_t)outer;
    } else if (position < 0) {
        *left_spd  = (int16_t)outer;
        *right_spd = (int16_t)inner;
    } else {
        *left_spd  = (int16_t)tgt;
        *right_spd = (int16_t)tgt;
    }

    Ramp_Targets(left_spd, right_spd, step);
}

/* OLED 分摊刷新：每 10ms 只重画一个字符，18 个字符轮一遍 180ms。
 * 驱动每个字节单独一次 I2C（~80us），8x16 的字要 25 次 ≈ 2ms；一次画整屏要 ~16ms，主循环会丢拍 */
#define OLED_CELLS   18

// [0,6) 状态  [6,12) 时间 "SS.HHs"  [12,18) 已转过的弯 "CNR nn"
static char    oled_text[OLED_CELLS];
static uint8_t oled_cell = 0;

static void OLED_Refresh_Step(void)
{
    static const char *const label[] = {"[STOP]", "[RUN ]", "[LOOP]"};   // 按 Car_State_t 顺序
    uint32_t ms   = (car_state == STATE_STOP) ? stopwatch_final_ms : Get_Current_Elapsed();
    uint32_t sec  = (ms / 1000) % 100;
    uint32_t hund = (ms % 1000) / 10;
    uint8_t  i;

    for (i = 0; i < 6; i++)
        oled_text[i] = label[car_state][i];

    oled_text[6]  = (sec >= 10) ? (char)('0' + sec / 10) : ' ';
    oled_text[7]  = (char)('0' + sec % 10);
    oled_text[8]  = '.';
    oled_text[9]  = (char)('0' + hund / 10);   // 不能消零：3.07s 会显示成 " 3. 7s"
    oled_text[10] = (char)('0' + hund % 10);
    oled_text[11] = 's';

    oled_text[12] = 'C';
    oled_text[13] = 'N';
    oled_text[14] = 'R';
    oled_text[15] = ' ';
    oled_text[16] = (corner_cnt >= 10) ? (char)('0' + corner_cnt / 10 % 10) : ' ';
    oled_text[17] = (char)('0' + corner_cnt % 10);

    i = oled_cell;
    if (i < 6)
        OLED_ShowChar(6 * i, 0, (uint8_t)oled_text[i], 8);
    else if (i < 12)
        OLED_ShowChar(8 + 8 * (i - 6), 3, (uint8_t)oled_text[i], 16);
    else
        OLED_ShowChar(6 * (i - 12), 6, (uint8_t)oled_text[i], 8);

    oled_cell = (uint8_t)((i + 1) % OLED_CELLS);
}

// 手写整数转十进制，避免为几行调试输出拖进整个格式化库
static char *Tele_Append_Int(char *p, int32_t v)
{
    char tmp[12];
    uint8_t n = 0;
    uint32_t u;

    if (v < 0) {
        *p++ = '-';
        u = (uint32_t)(-(v + 1)) + 1u;   // 写成这样是因为 v = INT32_MIN 时 -v 会溢出
    } else {
        u = (uint32_t)v;
    }

    do {
        tmp[n++] = (char)('0' + (u % 10));
        u /= 10;
    } while (u != 0);

    while (n != 0)
        *p++ = tmp[--n];

    return p;
}

/* 每行：A目标,A实际,B目标,B实际,C目标,C实际,D目标,D实际,灰度(8字符),raw0..raw7
 * 目标和实际都是细分单位（×SPEED_SCALE，即 0.1 计数/10ms），两者可直接比
 * 灰度第 1 个字符 = digital[0] = 小车最右侧那一路，全 0 = 全白丢线 */
static void Send_Wheel_Telemetry(void)
{
    char buf[128];
    char *p = buf;
    int16_t tgt[4];
    int16_t act[4];
    uint8_t i;

    /* 快照要关中断读齐：wheel_target_abcd 由 10ms ISR 更新，
     * 一个一个读会被 ISR 插进来凑出"半新半旧"的一行 */
    __disable_irq();
    for (i = 0; i < 4; i++)
        tgt[i] = wheel_target_abcd[i];

    act[0] = (int16_t)(encoderA_cnt * SPEED_SCALE);
    act[1] = (int16_t)(encoderB_cnt * SPEED_SCALE);
    act[2] = (int16_t)(encoderC_cnt * SPEED_SCALE);
    act[3] = (int16_t)(encoderD_cnt * SPEED_SCALE);
    __enable_irq();

    for (i = 0; i < 4; i++) {
        if (i != 0) *p++ = ',';
        p = Tele_Append_Int(p, tgt[i]);
        *p++ = ',';
        p = Tele_Append_Int(p, act[i]);
    }

    *p++ = ',';
    for (i = 0; i < GRAY_CHANNEL_COUNT; i++)
        *p++ = gray_sensor.digital[i] ? '1' : '0';

#if TELEMETRY_RAW
    for (i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        *p++ = ',';
        p = Tele_Append_Int(p, (int32_t)gray_sensor.raw[i]);
    }
#endif

    *p++ = '\r';
    *p++ = '\n';
    *p   = '\0';

    uart0_send_string_nb(buf);
}
