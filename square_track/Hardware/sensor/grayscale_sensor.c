#include "grayscale_sensor.h"

// 0 = 白底黑线；1 = 黑底白线（当前赛道），压线时 digital = 1
#define SENSOR_INVERT   1

#define GRAY_SETTLE_US      50
#define GRAY_SETTLE_CYCLES  ((CPUCLK_FREQ / 1000000u) * GRAY_SETTLE_US)

#define GRAY_ADC_TIMEOUT    2000

// 二值化判据，单位是 12 位 ADC 原始值（压线 ~4095、地面 ~146）
#define GRAY_CONTRAST_MIN   100    // 8 路 max-min 小于它 → 这一帧没有"线 vs 地面"
#define GRAY_ONLINE_LEVEL   1000   // 连最暗的一路都在这个电平之上 → 整条压在线上
#define GRAY_THRESH_HI      56     // 归一化百分比：≥56 判"线"
#define GRAY_THRESH_LO      44     // ≤44 判"地面"，中间保持上次（滞回）

typedef struct {
    GPIO_Regs *port;
    uint32_t   pin;
} MuxAddrLine;

static const MuxAddrLine mux_addr[3] = {
    { SENSOR_MUX_AD0_PORT, SENSOR_MUX_AD0_PIN },   // AD0 最低位
    { SENSOR_MUX_AD1_PORT, SENSOR_MUX_AD1_PIN },
    { SENSOR_MUX_AD2_PORT, SENSOR_MUX_AD2_PIN },
};

static void GraySensor_SelectChannel(uint8_t channel)
{
    uint8_t i;

    for (i = 0; i < 3; i++) {
        if (channel & (1u << i))
            DL_GPIO_setPins(mux_addr[i].port, mux_addr[i].pin);
        else
            DL_GPIO_clearPins(mux_addr[i].port, mux_addr[i].pin);
    }

    delay_cycles(GRAY_SETTLE_CYCLES);
}

#ifdef ADC12_0_INST

// repeat 模式连续转换，轮询结果标志即可；采 4 次取平均压噪声
static uint16_t GraySensor_ReadADC(void)
{
    uint32_t sum = 0;
    uint8_t k;

    for (k = 0; k < 4; k++) {
        uint32_t timeout = 0;
        while (!DL_ADC12_getRawInterruptStatus(
                   ADC12_0_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED)) {
            if (++timeout > GRAY_ADC_TIMEOUT) {
                return 0xFFFF;   // 无效值
            }
        }

        sum += DL_ADC12_getMemResult(ADC12_0_INST, DL_ADC12_MEM_IDX_0);

        DL_ADC12_clearInterruptStatus(
            ADC12_0_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    }

    return (uint16_t)(sum / 4);
}

#endif /* ADC12_0_INST */

void GraySensor_Init(GraySensor_Handle *sensor)
{
    uint8_t i;

    for (i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        sensor->raw[i]     = 0;
        sensor->level[i]   = 0;
        sensor->digital[i] = 0;
    }
    sensor->err_cnt = 0;

    DL_GPIO_clearPins(SENSOR_MUX_AD0_PORT, SENSOR_MUX_AD0_PIN);
    DL_GPIO_clearPins(SENSOR_MUX_AD1_PORT, SENSOR_MUX_AD1_PIN);
    DL_GPIO_clearPins(SENSOR_MUX_AD2_PORT, SENSOR_MUX_AD2_PIN);

    /* 下面两行缺一不可：SysConfig 既不会配 PA27 的模拟功能，
     * 也不会启动 repeat 模式的转换序列，少了就是读数恒 0 / 永远超时 */
    DL_GPIO_initPeripheralAnalogFunction(GPIO_ADC12_0_IOMUX_C0);
    DL_ADC12_startConversion(ADC12_0_INST);
}

#ifdef ADC12_0_INST

uint16_t GraySensor_ReadChannel(uint8_t channel)
{
    if (channel >= GRAY_CHANNEL_COUNT) {
        return 0;
    }

    GraySensor_SelectChannel(channel);
    return GraySensor_ReadADC();
}

void GraySensor_ScanAll(GraySensor_Handle *sensor)
{
    uint8_t i;
    uint16_t v;

    for (i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        v = GraySensor_ReadChannel(i);
        /* 0xFFFF 绝不能存进 raw[]：会把归一化的 rmax 抬到 65535，
         * 这一路变成"最白"、经反相后凭空造出一条线让车急转。
         * 沿用上一帧的值，但要计数，别把硬件故障掩盖掉 */
        if (v != 0xFFFF) {
            sensor->raw[i] = v;
        } else {
            sensor->err_cnt++;
        }
    }
}

void GraySensor_Binarize(GraySensor_Handle *sensor)
{
    uint8_t i;
    uint16_t rmax = 0, rmin = 4095;

    for (i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        if (sensor->raw[i] > rmax) rmax = sensor->raw[i];
        if (sensor->raw[i] < rmin) rmin = sensor->raw[i];
    }

    if ((uint16_t)(rmax - rmin) < GRAY_CONTRAST_MIN) {
        /* 展度小有两种相反的情况，只看展度会把它们混成一种：
         *   全在地面上（都暗）→ 真丢线；全压在宽线上（都亮）→ 应直行。
         * 用绝对电平区分，否则车压过宽线会被判成丢线直接甩出去 */
        uint16_t line_level = SENSOR_INVERT ? rmin : (uint16_t)(4095 - rmax);
        uint8_t  all_on_line = (line_level >= GRAY_ONLINE_LEVEL) ? 1 : 0;

        for (i = 0; i < GRAY_CHANNEL_COUNT; i++) {
            sensor->digital[i] = all_on_line ? 1 : 0;
            sensor->level[i]   = all_on_line ? 100 : 0;
        }
        return;
    }

    uint16_t range = (uint16_t)(rmax - rmin);
    for (i = 0; i < GRAY_CHANNEL_COUNT; i++) {
        uint16_t val = (uint16_t)(((uint32_t)(sensor->raw[i] - rmin) * 100u) / range);

        sensor->level[i] = (uint8_t)(SENSOR_INVERT ? val : 100u - val);

        if (val >= GRAY_THRESH_HI) {
            sensor->digital[i] = SENSOR_INVERT ? 1 : 0;
        } else if (val <= GRAY_THRESH_LO) {
            sensor->digital[i] = SENSOR_INVERT ? 0 : 1;
        }
        // 中间地带不动 digital[i]（滞回）
    }
}

void GraySensor_Update(GraySensor_Handle *sensor)
{
    GraySensor_ScanAll(sensor);
    GraySensor_Binarize(sensor);
}

#endif /* ADC12_0_INST */
