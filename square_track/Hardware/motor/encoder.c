/**
 * @file    encoder.c
 * @brief   四路正交编码器脉冲计数（GPIO 上升沿中断 + 另一相电平判向）
 * @note    中断里累加计数，10ms 定时器中断里读取并清零（见 main.c）
 */

#include "ti_msp_dl_config.h"
#include "encoder.h"

/* ── 编码器脉冲计数器（GPIO 中断里累加，定时器中断里读到主循环） ── */
volatile int32_t Get_Encoder_countA = 0;
volatile int32_t Get_Encoder_countB = 0;
volatile int32_t Get_Encoder_countC = 0;
volatile int32_t Get_Encoder_countD = 0;

/* 单路正交编码器处理：A/B 相的上升沿都触发，通过另一相的电平判断方向 */
#define QUAD_DECODE(count, port, a_pin, b_pin, intr)  \
    do {                                               \
        if ((intr) & (a_pin)) {                        \
            if (!DL_GPIO_readPins(port, b_pin))        \
                count--;                               \
            else                                       \
                count++;                               \
        }                                              \
        if ((intr) & (b_pin)) {                        \
            if (!DL_GPIO_readPins(port, a_pin))        \
                count++;                               \
            else                                       \
                count--;                               \
        }                                              \
    } while (0)

void GROUP1_IRQHandler(void)
{
    uint32_t gpioa_int = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    QUAD_DECODE(Get_Encoder_countA, GPIOA, ENCODERA_E1A_PIN, ENCODERA_E1B_PIN, gpioa_int);
    QUAD_DECODE(Get_Encoder_countB, GPIOA, ENCODERB_E2A_PIN, ENCODERB_E2B_PIN, gpioa_int);

    DL_GPIO_clearInterruptStatus(GPIOA,
        ENCODERA_E1A_PIN | ENCODERA_E1B_PIN | ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    uint32_t gpiob_int = DL_GPIO_getEnabledInterruptStatus(GPIOB,
        ENCODERC_PIN_1_PIN | ENCODERC_PIN_2_PIN | ENCODERD_PIN_3_PIN | ENCODERD_PIN_4_PIN);

    QUAD_DECODE(Get_Encoder_countC, GPIOB, ENCODERC_PIN_1_PIN, ENCODERC_PIN_2_PIN, gpiob_int);
    QUAD_DECODE(Get_Encoder_countD, GPIOB, ENCODERD_PIN_3_PIN, ENCODERD_PIN_4_PIN, gpiob_int);

    DL_GPIO_clearInterruptStatus(GPIOB,
        ENCODERC_PIN_1_PIN | ENCODERC_PIN_2_PIN | ENCODERD_PIN_3_PIN | ENCODERD_PIN_4_PIN);
}
