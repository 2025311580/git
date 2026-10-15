/**
 * @file    key.c
 * @brief   三颗按键驱动：电平读取 + 单击检测（非阻塞状态机，50ms 消抖）
 * @note    key*_click() 需每 10ms 调用一次（跟随主循环节拍）
 */

#include "key.h"

/* ── 电平读取 ── */
uint8_t key1_pressed(void)
{
    return (DL_GPIO_readPins(KEY_button1_PORT, KEY_button1_PIN) == 0);
}

uint8_t key2_pressed(void)
{
    return (DL_GPIO_readPins(KEY_button2_PORT, KEY_button2_PIN) == 0);
}

uint8_t key3_pressed(void)
{
    return (DL_GPIO_readPins(KEY_button3_PORT, KEY_button3_PIN) == 0);
}

static uint8_t key_pressed_by_index(uint8_t idx)
{
    switch (idx) {
        case 0:  return key1_pressed();
        case 1:  return key2_pressed();
        default: return key3_pressed();
    }
}

/* ── 单击检测：非阻塞状态机，每 10ms 调用一次，带 50ms 消抖 ──
 * 三颗按键共用同一套状态机（行为与原三份复制粘贴代码完全一致）：
 *   按下消抖 50ms → 确认按下 → 等待松开消抖 50ms → 返回单击 */
static uint8_t key_click_by_index(uint8_t idx)
{
    static uint8_t state[3] = {0, 0, 0};
    static uint8_t cnt[3]   = {0, 0, 0};
    uint8_t pressed = key_pressed_by_index(idx);

    switch (state[idx]) {
        case 0:
            if (pressed) { cnt[idx] = 5; state[idx] = 1; }
            break;
        case 1:
            if (!pressed) { state[idx] = 0; }
            else if (--cnt[idx] == 0) { state[idx] = 2; }
            break;
        case 2:
            if (!pressed) { cnt[idx] = 5; state[idx] = 3; }
            break;
        case 3:
            if (pressed) { state[idx] = 2; }
            else if (--cnt[idx] == 0) { state[idx] = 0; return 1; }
            break;
    }
    return 0;
}

uint8_t key1_click(void) { return key_click_by_index(0); }
uint8_t key2_click(void) { return key_click_by_index(1); }
uint8_t key3_click(void) { return key_click_by_index(2); }
