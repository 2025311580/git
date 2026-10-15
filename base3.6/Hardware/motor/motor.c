#include "ti_msp_dl_config.h"
#include "motor.h"
#include "common.h"
#include <stdlib.h>

#define VELOCITY_KP           2
#define VELOCITY_KI           1
#define VELOCITY_PWM_PER_UNIT  19
#define PID_MAX_OUT           300
#define MAX_INTEGRAL_COMPENSATE 40

// 电机线焊反了就改对应的这一位（当前四轮实测都是 0）
#define MOTOR_A_REVERSE    0
#define MOTOR_B_REVERSE    0
#define MOTOR_C_REVERSE    0
#define MOTOR_D_REVERSE    0

static int32_t integral[4] = {0};

// 启动斜坡期间置 0，冻结积分
static uint8_t integral_enable = 1;

void Motor_Set_Integral_Enable(uint8_t en)
{
    integral_enable = en;
}

static int32_t pid_calc(int32_t target, int32_t current, int32_t *integral)
{
    int32_t base_pwm = target * VELOCITY_PWM_PER_UNIT;

    int32_t err = target - current;

    if (target != 0) {
        if (integral_enable) {
            *integral += err;
        }
        *integral = CLAMP(*integral, -MAX_INTEGRAL_COMPENSATE, MAX_INTEGRAL_COMPENSATE);
    } else {
        *integral = 0;   // 内轮制动时必须清零而不是冻结，残留积分会把内轮顶出去
    }

    int32_t compensate = VELOCITY_KP * err + VELOCITY_KI * (*integral);
    compensate = CLAMP(compensate, -MAX_INTEGRAL_COMPENSATE, MAX_INTEGRAL_COMPENSATE);

    int32_t pwm = base_pwm + compensate;
    pwm = CLAMP(pwm, -PID_MAX_OUT, PID_MAX_OUT);

    return pwm;
}

int16_t Velocity_A(int16_t t, int16_t c) { return (int16_t)pid_calc(t, c, &integral[0]); }
int16_t Velocity_B(int16_t t, int16_t c) { return (int16_t)pid_calc(t, c, &integral[1]); }
int16_t Velocity_C(int16_t t, int16_t c) { return (int16_t)pid_calc(t, c, &integral[2]); }
int16_t Velocity_D(int16_t t, int16_t c) { return (int16_t)pid_calc(t, c, &integral[3]); }

void Reset_Motor_Integrals(void)
{
    integral[0] = 0;
    integral[1] = 0;
    integral[2] = 0;
    integral[3] = 0;
}

// pwm > 0 → IN1 高 IN2 低（正转）；< 0 反转；== 0 两路都高 = 主动刹车
static void set_single_motor_pwm(int pwm, uint8_t reverse,
                                 GPIO_Regs *in1_port, uint32_t in1_pin,
                                 GPIO_Regs *in2_port, uint32_t in2_pin,
                                 uint32_t pwm_channel)
{
    if(reverse)
    {
        pwm = -pwm;
    }

    if (pwm > 0)
    {
        DL_GPIO_setPins(in1_port, in1_pin);
        DL_GPIO_clearPins(in2_port, in2_pin);
    }
    else if (pwm < 0)
    {
        DL_GPIO_setPins(in2_port, in2_pin);
        DL_GPIO_clearPins(in1_port, in1_pin);
    }
    else
    {
        DL_GPIO_setPins(in1_port, in1_pin);
        DL_GPIO_setPins(in2_port, in2_pin);
    }

    DL_TimerA_setCaptureCompareValue(PWM_0_INST, abs(pwm), pwm_channel);
}

void Set_PWM(int16_t pwma, int16_t pwmb, int16_t pwmc, int16_t pwmd)
{
    set_single_motor_pwm(pwma, MOTOR_A_REVERSE, AIN1_PORT, AIN1_PIN_12_PIN, AIN2_PORT, AIN2_PIN_13_PIN, GPIO_PWM_0_C0_IDX);
    set_single_motor_pwm(pwmb, MOTOR_B_REVERSE, BIN1_PORT, BIN1_Pin_Bin1_PIN, BIN2_PORT, BIN2_Pin_Bin2_PIN, GPIO_PWM_0_C1_IDX);
    set_single_motor_pwm(pwmc, MOTOR_C_REVERSE, CIN1_PORT, CIN1_PIN_5_PIN, CIN2_PORT, CIN2_PIN_6_PIN, GPIO_PWM_0_C2_IDX);
    set_single_motor_pwm(pwmd, MOTOR_D_REVERSE, DIN1_PORT, DIN1_PIN_7_PIN, DIN2_PORT, DIN2_PIN_8_PIN, GPIO_PWM_0_C3_IDX);
}
