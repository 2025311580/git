/**
 * @file    motor.h
 * @brief   四路电机 PI 速度环与 PWM 输出接口
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 目标速度的细分：目标单位 = 1/SPEED_SCALE 个编码器计数/10ms，实际速度仍是整计数。
 * 不细分的话左右轮最小只能差 1 个计数（巡航 6 时就是 17%），线附近的小偏差全被取整成 0 */
#define SPEED_SCALE  10

// 四个电机的速度 PI 控制：传目标速度（细分单位）和实际速度（计数）进去，返回 PWM 值
int16_t Velocity_A(int16_t TargetVelocity, int16_t CurrentVelocity);
int16_t Velocity_B(int16_t TargetVelocity, int16_t CurrentVelocity);
int16_t Velocity_C(int16_t TargetVelocity, int16_t CurrentVelocity);
int16_t Velocity_D(int16_t TargetVelocity, int16_t CurrentVelocity);

// 转弯前调用：把四个电机的积分清零，避免旧方向的积分拖慢转向
void Reset_Motor_Integrals(void);

// 斜坡期间冻结/恢复积分累加（1=累加，0=冻结），防止启动超调
void Motor_Set_Integral_Enable(uint8_t en);

// 一口气把四个电机的 PWM 都设好
void Set_PWM(int16_t pwma, int16_t pwmb, int16_t pwmc, int16_t pwmd);

#ifdef __cplusplus
}
#endif

#endif