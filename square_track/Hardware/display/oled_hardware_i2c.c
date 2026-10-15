/**
 * @file    oled_hardware_i2c.c
 * @brief   SSD1306 OLED 驱动实现（硬件 I2C，含 SDA 卡死自恢复）
 * @note    I2C 传输带超时保护：超时后自动 bit-bang 解锁 SDA 并复位 I2C，
 *          依赖 system_ms（10ms 定时器中断累加）做超时基准
 */

#include "oled_hardware_i2c.h"
#include "oledfont.h"
#include "system/board.h"

/* ★必须 ≥ 20：system_ms 每 10ms 才跳一次、一跳 +10，写 10 的话传输途中
 * 碰上一次定时器中断就被判超时，白白复位 I2C（≥2ms）还打断正在写的字节 */
#define I2C_TIMEOUT_MS  (20)

extern volatile uint32_t system_ms;

static int mspm0_i2c_disable(void)
{
    DL_I2C_reset(I2C_0_INST);
    DL_GPIO_initDigitalOutput(GPIO_I2C_0_IOMUX_SCL);
    DL_GPIO_initDigitalInputFeatures(GPIO_I2C_0_IOMUX_SDA,
         DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_NONE,
         DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    DL_GPIO_enableOutput(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
    return 0;
}

static int mspm0_i2c_enable(void)
{
    DL_I2C_reset(I2C_0_INST);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SDA,
        GPIO_I2C_0_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_0_IOMUX_SCL,
        GPIO_I2C_0_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_0_IOMUX_SCL);
    DL_I2C_enablePower(I2C_0_INST);
    SYSCFG_DL_I2C_0_init();
    return 0;
}

void oled_i2c_sda_unlock(void)
{
    uint8_t cycleCnt = 0;
    mspm0_i2c_disable();
    do
    {
        DL_GPIO_clearPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_ms(1);
        DL_GPIO_setPins(GPIO_I2C_0_SCL_PORT, GPIO_I2C_0_SCL_PIN);
        delay_ms(1);

        if(DL_GPIO_readPins(GPIO_I2C_0_SDA_PORT, GPIO_I2C_0_SDA_PIN))
            break;
    }while(++cycleCnt < 100);
    mspm0_i2c_enable();
}

void OLED_ColorTurn(uint8_t i)
{
    if(i==0)
        OLED_WR_Byte(0xA6, OLED_CMD);
    if(i==1)
        OLED_WR_Byte(0xA7, OLED_CMD);
}

void OLED_DisplayTurn(uint8_t i)
{
    if(i==0)
    {
        OLED_WR_Byte(0xC8, OLED_CMD);
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if(i==1)
    {
        OLED_WR_Byte(0xC0, OLED_CMD);
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    unsigned char ptr[2];
    uint32_t start;

    if(mode)
    {
        ptr[0] = 0x40;
        ptr[1] = dat;
    }
    else
    {
        ptr[0] = 0x00;
        ptr[1] = dat;
    }

    start = system_ms;

    DL_I2C_fillControllerTXFIFO(I2C_0_INST, ptr, 2);
    DL_I2C_clearInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    uint32_t idle_timeout = 0;
    while (!(DL_I2C_getControllerStatus(I2C_0_INST) & DL_I2C_CONTROLLER_STATUS_IDLE)) {
        if (++idle_timeout > 100000) break;   // 防死等：控制器一直不空闲则放弃
    }
    DL_I2C_startControllerTransfer(I2C_0_INST, 0x3C, DL_I2C_CONTROLLER_DIRECTION_TX, 2);

    while (!DL_I2C_getRawInterruptStatus(I2C_0_INST, DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        if (system_ms - start >= I2C_TIMEOUT_MS)
        {
            oled_i2c_sda_unlock();
            break;
        }
    }
}

void OLED_Set_Pos(uint8_t x, uint8_t y)
{
    OLED_WR_Byte(0xb0+y, OLED_CMD);
    OLED_WR_Byte(((x&0xf0)>>4)|0x10, OLED_CMD);
    OLED_WR_Byte((x&0x0f), OLED_CMD);
}

void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);
    OLED_WR_Byte(0X14, OLED_CMD);
    OLED_WR_Byte(0XAF, OLED_CMD);
}

void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);
    OLED_WR_Byte(0X10, OLED_CMD);
    OLED_WR_Byte(0XAE, OLED_CMD);
}

void OLED_Clear(void)
{
    uint8_t i,n;
    for(i=0;i<8;i++)
    {
        OLED_WR_Byte(0xb0+i, OLED_CMD);
        OLED_WR_Byte(0x00, OLED_CMD);
        OLED_WR_Byte(0x10, OLED_CMD);
        for(n=0;n<128;n++) OLED_WR_Byte(0, OLED_DATA);
    }
}

void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{
    uint8_t c=0, sizex=sizey/2;
    uint16_t i=0, size1;
    if(sizey==8) size1=6;
    else size1=(sizey/8+((sizey%8)?1:0))*(sizey/2);
    c=chr-' ';
    OLED_Set_Pos(x,y);
    for(i=0;i<size1;i++)
    {
        if(i%sizex==0 && sizey!=8) OLED_Set_Pos(x,y++);
        if(sizey==8) OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);
        else if(sizey==16) OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);
        else return;
    }
}

uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result=1;
    while(n--) result*=m;
    return result;
}

void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{
    uint8_t t,temp,m=0;
    uint8_t enshow=0;
    if(sizey==8) m=2;
    for(t=0;t<len;t++)
    {
        temp=(num/oled_pow(10,len-t-1))%10;
        if(enshow==0 && t<(len-1))
        {
            if(temp==0)
            {
                OLED_ShowChar(x+(sizey/2+m)*t,y,' ',sizey);
                continue;
            }else enshow=1;
        }
        OLED_ShowChar(x+(sizey/2+m)*t,y,temp+'0',sizey);
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey)
{
    uint8_t j=0;
    while (chr[j]!='\0')
    {
        OLED_ShowChar(x,y,chr[j++],sizey);
        if(sizey==8) x+=6;
        else x+=sizey/2;
    }
}

void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)
{
    uint16_t i, size1=(sizey/8+((sizey%8)?1:0))*sizey;
    for(i=0;i<size1;i++)
    {
        if(i%sizey==0) OLED_Set_Pos(x,y++);
        if(sizey==16) OLED_WR_Byte(Hzk[no][i], OLED_DATA);
        else return;
    }
}

void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[])
{
    uint16_t j=0;
    uint8_t i,m;
    sizey=sizey/8+((sizey%8)?1:0);
    for(i=0;i<sizey;i++)
    {
        OLED_Set_Pos(x,i+y);
        for(m=0;m<sizex;m++)
        {
            OLED_WR_Byte(BMP[j++], OLED_DATA);
        }
    }
}

void OLED_Init(void)
{
    if(DL_I2C_getSDAStatus(I2C_0_INST) == DL_I2C_CONTROLLER_SDA_LOW)
        oled_i2c_sda_unlock();

    delay_ms(200);

    OLED_WR_Byte(0xAE, OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD);//---set low column address
    OLED_WR_Byte(0x10, OLED_CMD);//---set high column address
    OLED_WR_Byte(0x40, OLED_CMD);//--set start line address
    OLED_WR_Byte(0x81, OLED_CMD);//--set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD);//--Set SEG/Column Mapping
    OLED_WR_Byte(0xC8, OLED_CMD);//Set COM/Row Scan Direction
    OLED_WR_Byte(0xA6, OLED_CMD);//--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD);//--set multiplex ratio(1 to 64)
    OLED_WR_Byte(0x3f, OLED_CMD);//--1/64 duty
    OLED_WR_Byte(0xD3, OLED_CMD);//-set display offset
    OLED_WR_Byte(0x00, OLED_CMD);//-not offset
    OLED_WR_Byte(0xd5, OLED_CMD);//--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD);//--set divide ratio
    OLED_WR_Byte(0xD9, OLED_CMD);//--set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD);//--set com pins hardware configuration
    OLED_WR_Byte(0x12, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD);//--set vcomh
    OLED_WR_Byte(0x40, OLED_CMD);//Set VCOM Deselect Level
    OLED_WR_Byte(0x20, OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD);//
    OLED_WR_Byte(0x8D, OLED_CMD);//--set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD);//--set(0x10) disable
    OLED_WR_Byte(0xA4, OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6, OLED_CMD);// Disable Inverse Display On (0xa6/a7)
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/
}
