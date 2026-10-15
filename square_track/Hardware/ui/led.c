/**
 * @file    led.c
 * @brief   板载 LED 控制：亮 / 灭 / 翻转 / 闪烁
 * @note    当前主程序未接入，留作调试指示灯备用
 */

#include "led.h"

/* ── LED 基本操作 ── */

void LED_ON(void)
{
    DL_GPIO_clearPins(LED1_PORT, LED1_PIN_0_PIN);    // 低电平亮
}

void LED_OFF(void)
{
    DL_GPIO_setPins(LED1_PORT, LED1_PIN_0_PIN);      // 高电平灭
}

void LED_Toggle(void)
{
    DL_GPIO_togglePins(LED1_PORT, LED1_PIN_0_PIN);   // 翻转一下
}

// LED 闪烁：time=0 常亮，time>0 时每隔 time 次调用翻转一次
void LED_Flash(uint16_t time)
{
    static uint16_t temp;
    if (time == 0)
        LED_ON();
    else if (++temp == time)
        LED_Toggle(), temp = 0;
}
