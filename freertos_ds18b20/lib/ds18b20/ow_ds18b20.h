/**
 * @file ow_ds18b20.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-12-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _OW_DS18B20_HEAD_H_
#define _OW_DS18B20_HEAD_H_

#include "stdint.h"
#include "delay.h"

typedef struct _ow{
    uint32_t port;
    uint16_t pin;
    uint32_t temp;
}ow_t;

int ow_ds18b20_init(ow_t obj[], uint8_t num);
int ow_ds18b20_conv_temp(ow_t obj[], uint8_t num);
int ow_ds18b20_get_temp(ow_t obj[], uint8_t num);

#endif //!_OW_DS18B20_HEAD_H_