#ifndef _UART_CALLBACK_H_
#define _UART_CALLBACK_H_
#include "board.h"

/* 这四个是 UART1 接收中断里写的、主循环里读的，必须 volatile —— 否则编译器
 * 有权把它们缓存进寄存器，主循环就永远看到启动时那个值。现在 -O0 下不出问题
 * 只是因为每次都要重新访存，改优化等级或开 LTO 就会咬人。 */
extern volatile int motor1_Current_Speed, motor2_Current_Speed;
extern volatile int Motor1_Current_Position, Motor2_Current_Position;

/* 下面两个是主循环写的（目标速度、给外部工具抓的波形），不跨中断，不用 volatile */
extern int Motor1_Speed, Motor2_Speed;
extern int Motor1_T_Position, Motor2_T_Position;

void usart1_send(u8 data);
void USART1_SEND(u8 *data, u8 len);
u8 BCC_Sum1(u8 *usart_data, unsigned char Count_Number);

#endif