/**
 * @file    key.h
 * @brief   按键驱动接口：电平读取 + 单击检测（50ms 消抖状态机）
 */

#ifndef _KEY_H
#define _KEY_H
#include "ti_msp_dl_config.h"

// 按键按下检测（直接读电平）
uint8_t key1_pressed(void);
uint8_t key2_pressed(void);
uint8_t key3_pressed(void);

// 单击检测：非阻塞状态机，每 10ms 调用一次，带 50ms 消抖
uint8_t key1_click(void);
uint8_t key2_click(void);
uint8_t key3_click(void);

#endif
