/**
 * @file    led.h
 * @brief   板载 LED 控制接口（当前备用，主程序未接入）
 */

#ifndef _LED_H
#define _LED_H
#include "ti_msp_dl_config.h"

// LED 控制：亮、灭、翻转、闪烁
void LED_ON(void);
void LED_OFF(void);
void LED_Toggle(void);
void LED_Flash(uint16_t time);    // 0=常亮, >0 时每 time 次调用翻转
#endif 
