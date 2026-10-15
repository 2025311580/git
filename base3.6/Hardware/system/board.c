#include "board.h"
#include "stdio.h"

#define RE_0_BUFF_LEN_MAX   128

volatile uint8_t  recv0_buff[RE_0_BUFF_LEN_MAX] = {0};
volatile uint16_t recv0_length = 0;
volatile uint8_t  recv0_flag = 0;

void delay_us(unsigned long __us)
{
    uint32_t ticks;
    uint32_t told, tnow, tcnt = 38;

    ticks = __us * 32;    // 32MHz 下 1μs = 32 tick
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;

        if (tnow != told)
        {
            if (tnow < told)
                tcnt += told - tnow;
            else
                tcnt += SysTick->LOAD - tnow + told;   // 重载，跨了一圈

            told = tnow;

            if (tcnt >= ticks)
                break;
        }
    }
}

void delay_ms(unsigned long ms)
{
    delay_us(ms * 1000);
}

void uart0_send_char(char ch)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitData(UART_0_INST, ch);
}

void uart0_send_string(char* str)
{
    if (str == NULL) return;
    while (*str != '\0')
    {
        uart0_send_char(*str++);
    }
}

/* 非阻塞发送：环形缓冲 + TX 中断。缓冲满就整行丢掉，绝不在发送上等 ——
 * 遥测是诊断输出，不能反过来阻塞 10ms 控制周期 */
#define UART0_TX_BUFF_LEN   256

static volatile uint8_t  tx_buff[UART0_TX_BUFF_LEN];
static volatile uint16_t tx_head = 0;
static volatile uint16_t tx_tail = 0;
static volatile uint8_t  tx_running = 0;

static void uart0_tx_kick(void)
{
    if (tx_running || tx_tail == tx_head)
        return;

    tx_running = 1;
    DL_UART_Main_transmitData(UART_0_INST, tx_buff[tx_tail]);
    tx_tail = (uint16_t)((tx_tail + 1) % UART0_TX_BUFF_LEN);
    DL_UART_enableInterrupt(UART_0_INST, DL_UART_INTERRUPT_TX);
}

void uart0_send_string_nb(const char *str)
{
    uint16_t len = 0, head, free_cnt;

    if (str == NULL)
        return;

    while (str[len] != '\0')
        len++;
    if (len == 0)
        return;

    __disable_irq();

    head = tx_head;
    free_cnt = (uint16_t)((tx_tail + UART0_TX_BUFF_LEN - head - 1) % UART0_TX_BUFF_LEN);
    if (len > free_cnt)
    {
        __enable_irq();
        return;
    }

    for (uint16_t i = 0; i < len; i++)
    {
        tx_buff[head] = (uint8_t)str[i];
        head = (uint16_t)((head + 1) % UART0_TX_BUFF_LEN);
    }
    tx_head = head;

    uart0_tx_kick();

    __enable_irq();
}

#if !defined(__MICROLIB)
#if (__ARMCLIB_VERSION <= 6000000)
struct __FILE
{
    int handle;
};
#endif

FILE __stdout;

void _sys_exit(int x)
{
    x = x;
}
#endif

int fputc(int ch, FILE *stream)
{
    while (DL_UART_isBusy(UART_0_INST) == true);
    DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);
    return ch;
}

int fputs(const char* restrict s, FILE* restrict stream)
{
    uint16_t count = 0;
    while (*s) {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, *s++);
        count++;
    }
    return count;
}

int puts(const char *_ptr)
{
    int count = fputs(_ptr, stdout);
    count += fputs("\n", stdout);
    return count;
}

void UART_0_INST_IRQHandler(void)
{
    uint8_t receivedData = 0;

    switch (DL_UART_getPendingInterrupt(UART_0_INST))
    {
        case DL_UART_IIDX_RX:

            receivedData = DL_UART_Main_receiveData(UART_0_INST);

            if (recv0_length < RE_0_BUFF_LEN_MAX - 1)
            {
                recv0_buff[recv0_length++] = receivedData;
            }
            else
            {
                recv0_length = 0;
            }
            recv0_flag = 1;
            break;

        case DL_UART_IIDX_TX:

            if (tx_tail != tx_head)
            {
                DL_UART_Main_transmitData(UART_0_INST, tx_buff[tx_tail]);
                tx_tail = (uint16_t)((tx_tail + 1) % UART0_TX_BUFF_LEN);
            }
            else
            {
                /* 必须等"真的空了"这一拍才关中断：写完最后一个字节就关，
                 * 下次 kick 会误判发送器空闲、覆写保持寄存器，字节就丢了 */
                DL_UART_disableInterrupt(UART_0_INST, DL_UART_INTERRUPT_TX);
                tx_running = 0;
            }
            break;

        default:
            break;
    }
}
