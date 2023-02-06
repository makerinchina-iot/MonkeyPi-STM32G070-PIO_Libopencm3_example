/**
 * @file packet.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2023-01-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef _PACKET_HEAD_H_
#define _PACKET_HEAD_H_

#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif

typedef void (*send_buff_fn)(uint8_t *buff, uint8_t len);

//shoud set thhi first
void    packet_wave_send_fn_set(send_buff_fn fn);

/**
 * @brief send wave data type u8
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_u8(uint8_t channel, uint8_t value);

/**
 * @brief send wave data type i8
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_i8(uint8_t channel, int8_t value);

/**
 * @brief send wave data type u16
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_u16(uint8_t channel, uint16_t value);

/**
 * @brief send wave data type i16
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_i16(uint8_t channel, int16_t value);

/**
 * @brief send wave data type u32
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_u32(uint8_t channel, uint32_t value);

/**
 * @brief send wave data type i32
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_i32(uint8_t channel, int32_t value);

/**
 * @brief send wave data type float
 * 
 * @param channel 
 * @param value 
 * @return uint8_t 
 */
uint8_t packet_wave_send_float(uint8_t channel, float value);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif //!_PACKET_HEAD_H_