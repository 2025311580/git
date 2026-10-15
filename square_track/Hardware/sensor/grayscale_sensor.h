#ifndef GRAYSCALE_SENSOR_H
#define GRAYSCALE_SENSOR_H

#include "ti_msp_dl_config.h"
#include <stdbool.h>
#include <stdint.h>

#define GRAY_CHANNEL_COUNT  8

typedef struct {
    uint16_t raw[GRAY_CHANNEL_COUNT];
    uint8_t  level[GRAY_CHANNEL_COUNT];     // 0~100 像线的程度（本帧归一化、已按 SENSOR_INVERT 换算），位置插值用
    uint8_t  digital[GRAY_CHANNEL_COUNT];   // digital[0] = 小车最右侧那一路
    uint16_t err_cnt;                       // ADC 超时次数，正常长期为 0
} GraySensor_Handle;

void     GraySensor_Init(GraySensor_Handle *sensor);
uint16_t GraySensor_ReadChannel(uint8_t channel);
void     GraySensor_ScanAll(GraySensor_Handle *sensor);

void     GraySensor_Binarize(GraySensor_Handle *sensor);
void     GraySensor_Update(GraySensor_Handle *sensor);

#endif /* GRAYSCALE_SENSOR_H */
