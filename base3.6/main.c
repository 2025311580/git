#include "ti_msp_dl_config.h"
#include "motor/motor.h"
#include "motor/encoder.h"
#include "sensor/grayscale_sensor.h"
#include <stdint.h>
#include <stdbool.h>
#include "display/oled_hardware_i2c.h"
#include "system/board.h"


volatile int32_t encoderA_cnt = 0, encoderB_cnt = 0, encoderC_cnt = 0, encoderD_cnt = 0;

#define SENSOR_NUM        8
#define TRACK_D_GAIN      12

/* 250 = 阵列边缘（|position| 最大才 350）。改回参考版的 100 会让转向
 * 饱和成三层继电器，实测 1.9Hz 极限环 —— 想更跟线可以降到 200，别低于 150 */
#define POS_DIVISOR       250
#define LEFT_TRIM         100    // 左轮机械补偿（%）
#define SPEED_RAMP         1     // 每 10ms 速度目标最大变化量
#define WIDE_RAMP_STEP      4    // 宽横线回正用的斜率，比正常转向陡

/* MAX_SPEED 由 PID 的输出上限反推：base_pwm = 速度 × VELOCITY_PWM_PER_UNIT(19)，
 * motor.c 里 PID_MAX_OUT = 300，所以 target ≥ 16 会被削成同一个占空比。
 * 取 15 是为了让 turn 曲线全程单调 —— 写 30 的话 turn ≥ 64 的那几档输出完全相同 */
#define NORM_TARGET_SPEED   8
#define NORM_MAX_SPEED      15
#define NORM_MIN_SPEED      2

#define SLOW_TARGET_SPEED   6
#define SLOW_MAX_SPEED      15
#define SLOW_MIN_SPEED      2

#define STARTUP_RAMP_MAX    100
#define STARTUP_RAMP_STEP   4     // 250ms 到满速
#define SLOW_RAMP_MAX       300
#define SLOW_RAMP_STEP      4     // 750ms 到满速；4 是实测的启动死区下限，别调小

#define TELEMETRY_ENABLE     1
#define TELEMETRY_RAW        1
#define TELEMETRY_EVERY_N    2     // 每 N 个 10ms 周期发一行（2 = 50Hz）

// 实测校准，勿改
#define ENCODER_AC_INVERT   1
#define ENCODER_BD_INVERT   0

typedef enum {
    STATE_STOP = 0,
    STATE_NORMAL,
    STATE_SLOW
} Car_State_t;

static GraySensor_Handle gray_sensor;
static Car_State_t car_state = STATE_STOP;

static int32_t track_last_error = 0;
static int32_t filtered_d = 0;
static int32_t smooth_pos = 0;
static int16_t smooth_l = 0, smooth_r = 0;

static uint32_t mode_entry_ms = 0;
static uint32_t stopwatch_final_ms = 0;
static uint8_t  oled_cnt = 0;

static volatile int16_t ctrl_target_l = 0;
static volatile int16_t ctrl_target_r = 0;
static volatile uint8_t ctrl_enable   = 0;
/* ★必须是 uint16_t：SLOW 的斜坡上限是 300，uint8_t 加到 255 会溢出回 0，
 * 表现为"跑几秒停一下"，且在 SLOW 下永远到不了满输出 */
static volatile uint16_t startup_ramp = 0;
static volatile uint16_t ctrl_ramp_max  = STARTUP_RAMP_MAX;
static volatile uint16_t ctrl_ramp_step = STARTUP_RAMP_STEP;

static volatile int16_t wheel_target_abcd[4] = {0};
static uint16_t tele_div = 0;

volatile uint32_t system_ms = 0;

volatile uint8_t  flag_10ms = 0;

static uint8_t Track_Line(int16_t *left_spd, int16_t *right_spd);
static void Ramp_Targets(int16_t *left_spd, int16_t *right_spd, int16_t step);
static void Reset_Track_State(void);
static int16_t Trim_Left(int16_t spd);
static void Apply_Motor_Speeds(int16_t left_spd, int16_t right_spd, uint16_t ramp, uint16_t ramp_max);
static void Reset_PWM_Filter(void);
static uint32_t Get_Current_Elapsed(void);
static void Enter_Mode(Car_State_t st);
static void OLED_Draw_Time(uint32_t ms, bool stopped);
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

    /* 无按键、无起终点：上电立刻进慢巡线，一直巡到断电为止。
     * Enter_Mode 内部会 OLED_Clear + 画 [SLOW]，并把 mode_entry_ms 对齐到
     * 当前 system_ms（此时≈0，秒表实际就是从开机算起） */
    Enter_Mode(STATE_SLOW);

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

            case STATE_NORMAL:
            case STATE_SLOW: {
                GraySensor_Update(&gray_sensor);

                /* 纯巡线，不判起终点：全白（宽横线，line_cnt ≥ 4）和全黑
                 * （丢线，line_cnt == 0）在 Track_Line 里都已经按"回正直行"
                 * 出参，返回值没有别的用处，直接丢弃 */
                Track_Line(&left_spd, &right_spd);

                ctrl_target_l = left_spd;
                ctrl_target_r = right_spd;
                ctrl_enable = 1;
                break;
            }

            /* 兜底：非法状态必须关速度环，否则 ISR 会带着上一拍的目标继续跑 */
            default:
                ctrl_enable = 0;
                break;
        }

#if TELEMETRY_ENABLE
        /* 遥测跟状态无关，三种状态下都发：停车时编码器差分自然≈0、四路目标是 0，
         * 正好能拿静止的车校准传感器（STOP 态也在刷 GraySensor_Update） */
        if (++tele_div >= TELEMETRY_EVERY_N) {
            tele_div = 0;
            Send_Wheel_Telemetry();
        }
#endif

        if (++oled_cnt >= 20) {
            oled_cnt = 0;
            OLED_Draw_Time((car_state == STATE_STOP) ? stopwatch_final_ms
                                                     : Get_Current_Elapsed(),
                           car_state == STATE_STOP);
        }

        /* 无按键、无终点线：进 STATE_SLOW 之后就再也出不来，唯一结束方式是断电 */
    }
}

static uint32_t Get_Current_Elapsed(void)
{
    return system_ms - mode_entry_ms;
}

// ★car_state 必须先赋值：OLED_Draw_Time 靠它区分 [RUN ] / [SLOW]
static void Enter_Mode(Car_State_t st)
{
    car_state = st;
    mode_entry_ms = system_ms;
    stopwatch_final_ms = 0;
    /* 斜坡参数随模式定一次即可：car_state 除了本函数，没有别处会改写 */
    ctrl_ramp_max  = (st == STATE_SLOW) ? SLOW_RAMP_MAX : STARTUP_RAMP_MAX;
    ctrl_ramp_step = (st == STATE_SLOW) ? SLOW_RAMP_STEP : STARTUP_RAMP_STEP;
    Reset_Motor_Integrals();
    Reset_PWM_Filter();
    Reset_Track_State();
    smooth_l = smooth_r = 0;
    startup_ramp = 0;
    oled_cnt = 0;
    OLED_Clear();
    OLED_Draw_Time(0, false);
}

/* ★Track_Line 的每一条出口都必须经过它（包括宽横线和丢线那两支）：
 * 它同时更新 smooth_l/r，漏掉的话限速器会拿过期的起点做基准，目标被反向弹一次 */
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

/* ★只在发车时（Enter_Mode）调用：丢线/宽线期间每拍清零的话，
 * 重新压线时 D 项会把整个 error 当成差分，车猛甩一下。
 * 这里也不清 smooth_l/r，那是速度斜坡的状态，由 Ramp_Targets 管 */
static void Reset_Track_State(void)
{
    track_last_error = 0;
    filtered_d = 0;
    smooth_pos = 0;
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

    /* ramp ≤ ramp_max 由 ISR 保证（startup_ramp 在那边已经钳过） */

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

        if (ctrl_enable) {
            if (startup_ramp < ctrl_ramp_max) {
                startup_ramp += ctrl_ramp_step;
                if (startup_ramp > ctrl_ramp_max)
                    startup_ramp = ctrl_ramp_max;
            }
            Apply_Motor_Speeds(ctrl_target_l, ctrl_target_r, startup_ramp, ctrl_ramp_max);
        } else {
            Set_PWM(0, 0, 0, 0);   // PWM=0 即双高制动（电机端子短接），不倒转
        }

        flag_10ms = 1;
    }
}

static uint8_t Track_Line(int16_t *left_spd, int16_t *right_spd)
{
    static const int8_t weight[SENSOR_NUM] = {-7, -4, -2, -1, 1, 2, 4, 7};

    int32_t error = 0;
    uint8_t line_cnt = 0;
    int32_t sum_w = 0, sum_wi = 0;   // 模拟重心：权重和、权重×通道号

    for (uint8_t i = 0; i < SENSOR_NUM; i++) {
        sum_w  += gray_sensor.norm[i];
        sum_wi += (int32_t)gray_sensor.norm[i] * (int32_t)i;

        if (gray_sensor.digital[i]) {
            line_cnt++;
            error += weight[i];
        }
    }

    int32_t tgt, max_s, min_s;
    if (car_state == STATE_NORMAL) {
        tgt   = NORM_TARGET_SPEED;
        max_s = NORM_MAX_SPEED;
        min_s = NORM_MIN_SPEED;
    } else {
        tgt   = SLOW_TARGET_SPEED;
        max_s = SLOW_MAX_SPEED;
        min_s = SLOW_MIN_SPEED;
    }

    /* ★这两个滤波器必须走在两个 early return 之前。盲区里位置不可知，就按"居中"
     * （error = 0 正是居中的取值）喂进去，让状态持续演进，否则会冻在进盲区那一刻：
     *   - track_last_error 跟上之后，重新压线时的差分才是真实的帧间差，而不是把
     *     整段盲区的位移当成一次差分，让 D 项凭空多出上百；
     *   - smooth_pos 不冻结，出盲区时车才不会带着进盲区前的旧转角再甩几拍 */
    int32_t err_now = (line_cnt == 0 || line_cnt >= 4) ? 0 : error;
    int32_t d_error = err_now - track_last_error;
    track_last_error = err_now;
    filtered_d = (filtered_d + d_error) / 2;

    // 宽横线（起步线/终点线）：位置算不出来，回正直行，由上层判是不是终点
    if (line_cnt >= 4) {
        *left_spd  = (int16_t)tgt;
        *right_spd = (int16_t)tgt;
        smooth_pos = (smooth_pos * 3) / 4;   // 向居中松弛，出线时不带旧转角
        Ramp_Targets(left_spd, right_spd, WIDE_RAMP_STEP);
        return line_cnt;
    }

    // 一个通道都没压到线（全黑）：回正直行找线
    if (line_cnt == 0) {
        *left_spd  = (int16_t)tgt;
        *right_spd = (int16_t)tgt;
        smooth_pos = (smooth_pos * 3) / 4;
        Ramp_Targets(left_spd, right_spd, SPEED_RAMP);
        return line_cnt;
    }

    /* 模拟重心（权重 0..100，范围同样是 -350..+350，POS_DIVISOR 的标定不受影响）。
     * 二值重心只能给出 i×100 的整数档，单路压线时 position 只有 ±50/±150/±250
     * 三个值 —— 线偏半个通道时左右轮差算出来是 0，中间那一段完全没反应。
     * 换成按模拟量加权后位置连续，这条死区就没了。
     * sum_w 恒 > 0：有对比度的帧里最亮那一路 norm = 100 ⇒ digital = 1 ⇒ line_cnt ≥ 1。
     * 除零兜底退回二值重心，正常走不到 */
    int32_t position;
    if (sum_w > 0) {
        position = sum_wi * 100 / sum_w - 350;
    } else {
        int32_t sum_pos = 0;
        for (uint8_t i = 0; i < SENSOR_NUM; i++)
            if (gray_sensor.digital[i]) sum_pos += (int32_t)i * 100;
        position = sum_pos / line_cnt - 350;
    }

    position += filtered_d * TRACK_D_GAIN;

    smooth_pos = (smooth_pos * 3 + position) / 4;
    position = smooth_pos;

    int32_t abs_pos = (position >= 0) ? position : -position;
    if (abs_pos > POS_DIVISOR) abs_pos = POS_DIVISOR;
    int32_t turn = abs_pos * abs_pos * 100 / (POS_DIVISOR * POS_DIVISOR);

    // turn ≤ 100 ⇒ inner ≥ min_s，不需要再钳位
    int32_t outer = tgt + (max_s - tgt) * turn / 100;
    int32_t inner = tgt - (tgt - min_s) * turn / 100;

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

    Ramp_Targets(left_spd, right_spd, SPEED_RAMP);

    return line_cnt;
}

static void OLED_Draw_Time(uint32_t ms, bool stopped)
{
    uint16_t sec  = (uint16_t)(ms / 1000);
    uint8_t  hund = (uint8_t)((ms % 1000) / 10);

    if (stopped)
        OLED_ShowString(0, 0, (uint8_t *)"[STOP]", 8);
    else if (car_state == STATE_SLOW)
        OLED_ShowString(0, 0, (uint8_t *)"[SLOW]", 8);
    else
        OLED_ShowString(0, 0, (uint8_t *)"[RUN ]", 8);

    OLED_ShowNum(8, 3, sec, 2, 16);
    OLED_ShowChar(8 + 16, 3, '.', 16);
    OLED_ShowNum(8 + 16 + 8, 3, hund, 2, 16);
    OLED_ShowChar(8 + 16 + 8 + 16, 3, 's', 16);
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
 * 灰度第 1 个字符 = digital[0] = 小车最右侧那一路，全 0 = 一个通道都没压到线 */
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

    act[0] = (int16_t)encoderA_cnt;
    act[1] = (int16_t)encoderB_cnt;
    act[2] = (int16_t)encoderC_cnt;
    act[3] = (int16_t)encoderD_cnt;
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
