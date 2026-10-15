/**
 * @file    encoder.h
 * @brief   四路正交编码器脉冲计数接口（GPIO 中断写入，定时器中断读取清零）
 */

#ifndef _ENCODER_H
#define _ENCODER_H
#include "ti_msp_dl_config.h"

// 四个编码器的实时脉冲计数（GPIO 中断写入，定时器中断读取清零）
extern volatile int32_t Get_Encoder_countA;
extern volatile int32_t Get_Encoder_countB;
extern volatile int32_t Get_Encoder_countC;
extern volatile int32_t Get_Encoder_countD;

#endif
