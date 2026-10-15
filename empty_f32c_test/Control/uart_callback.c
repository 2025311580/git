#include "ti_msp_dl_config.h"
#include "board.h"
#include "uart_callback.h"

/* UART1 接收缓存：电机反馈帧固定为 9 字节。 */
u8 usart1_receive_data[9];

/* UART1 发送 1 字节，等待发送 FIFO 有空位后写入。 */
void usart1_send(u8 data)
{
    while (DL_UART_isTXFIFOFull(UART_1_INST) == true);
    DL_UART_Main_transmitData(UART_1_INST, data);
}

/* UART1 发送数组，保持参考例程 USARTx_SEND 的调用风格。 */
void USART1_SEND(u8 *data, u8 len)
{
    u8 i = 0;

    for (i = 0; i < len; i++)
    {
        usart1_send(data[i]);
    }
}

/* BCC 异或校验：从帧头开始，对指定字节数逐字节异或。 */
u8 BCC_Sum1(u8 *usart_data, unsigned char Count_Number)
{
    unsigned char crc_sum = 0, k;

    for (k = 0; k < Count_Number; k++)
    {
        crc_sum = crc_sum ^ usart_data[k];
    }

    return crc_sum;
}

/* UART1 接收中断：逐字节拼接 9 字节反馈帧，校验通过后更新实时位置。 */
void UART_1_INST_IRQHandler(void)
{
    u8 Usart1_Receive;
    static u8 Count = 0;

    if (DL_UART_Main_getPendingInterrupt(UART_1_INST) == DL_UART_MAIN_IIDX_RX)
    {
        Usart1_Receive = DL_UART_Main_receiveData(UART_1_INST);

        /* 等待帧头 0x7A，避免错误字节导致接收错位。 */
        if (Count == 0 && Usart1_Receive != 0x7A)
        {
            return;
        }

        usart1_receive_data[Count++] = Usart1_Receive;

        if (Count >= 9)
        {
            Count = 0;

            /* 帧尾必须为 0x7B。 */
            if (usart1_receive_data[8] != 0x7B)
            {
                return;
            }

            /* 第 8 字节为前 7 字节 BCC 校验。 */
            if (usart1_receive_data[7] == BCC_Sum1(usart1_receive_data, 7))
            {
                u8  addr = usart1_receive_data[1];
                u8  type = usart1_receive_data[2];
                /* 数据段是 4 字节大端有符号数：多圈总角度可以是负数，所以要按
                 * int 还原。拼装必须先转 u32 再左移 —— data[3] 是 u8 提升成 int，
                 * 值 >= 0x80 时 (data[3] << 24) 直接溢出 int，是有符号左移 UB。 */
                int value = (int)(((u32)usart1_receive_data[3] << 24) |
                                  ((u32)usart1_receive_data[4] << 16) |
                                  ((u32)usart1_receive_data[5] << 8)  |
                                   (u32)usart1_receive_data[6]);

                /* 0x01 = 多圈总角度反馈，0x02 = 机械角度（单圈绝对）反馈，都按位置处理；
                 * 0x00 = 实时转速反馈。本工程只用到 0x02，其余照手册一并兼容。 */
                if (addr == 0x01)
                {
                    if (type == 0x01 || type == 0x02) Motor1_Current_Position = value;
                    else if (type == 0x00)            motor1_Current_Speed    = value;
                }
                else if (addr == 0x02)
                {
                    if (type == 0x01 || type == 0x02) Motor2_Current_Position = value;
                    else if (type == 0x00)            motor2_Current_Speed    = value;
                }
            }
        }
    }
}