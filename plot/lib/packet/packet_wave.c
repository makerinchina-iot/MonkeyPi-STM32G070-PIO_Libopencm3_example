/**
 * @file packet.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2023-01-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "packet_wave.h"
#include "cobs.h"

// #include <Arduino.h>

#define CHANNEL_MAX     15
#define PACKET_PLOT     0xA0

enum {
    PACKET_DATA_I8      = 0x10,
    PACKET_DATA_U8      = 0x11,
    PACKET_DATA_I16     = 0x12,
    PACKET_DATA_U16     = 0x13,
    PACKET_DATA_I32     = 0x14,
    PACKET_DATA_U32     = 0x15,
    PACKET_DATA_FLOAT   = 0x16,
};

#define PACKET_LEN_I8       (4+1)
#define PACKET_LEN_U8       (4+1)
#define PACKET_LEN_I16      (4+2)
#define PACKET_LEN_U16      (4+2)
#define PACKET_LEN_I32      (4+4)
#define PACKET_LEN_U32      (4+4)
#define PACKET_LEN_FLOAT    (4+4)

static send_buff_fn packet_send;

/**
 * @brief set the send data fn
 * 
 * @param fn 
 */
void    packet_wave_send_fn_set(send_buff_fn fn)
{
    packet_send = fn;
}

uint8_t packet_wave_send_u8(uint8_t channel, uint8_t value)
{
    uint8_t buff[PACKET_LEN_U8] = {0};
    uint8_t cobs_buff[PACKET_LEN_U8+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_U8;
    buff[2] = channel;
    buff[3] = PACKET_DATA_U8;
    buff[4] = value;


    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_U8);

    packet_send(cobs_buff,ret);
    
    return ret; 
}

uint8_t packet_wave_send_i8(uint8_t channel, int8_t value)
{
    uint8_t buff[PACKET_LEN_I8] = {0};
    uint8_t cobs_buff[PACKET_LEN_I8+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_I8;
    buff[2] = channel;
    buff[3] = PACKET_DATA_I8;
    buff[4] = value;


    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_I8);

    packet_send(cobs_buff,ret);
    
    return ret; 
}

uint8_t packet_wave_send_u16(uint8_t channel, uint16_t value)
{
    uint8_t buff[PACKET_LEN_U16] = {0};
    uint8_t cobs_buff[PACKET_LEN_U16+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_U16;
    buff[2] = channel;
    buff[3] = PACKET_DATA_U16;
    buff[4] = (value>>8)&0xff;
    buff[5] = value&0xff;


    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_U16);

    packet_send(cobs_buff,ret);
    
    return ret; 
}

uint8_t packet_wave_send_i16(uint8_t channel, int16_t value)
{
    uint8_t buff[PACKET_LEN_I16] = {0};
    uint8_t cobs_buff[PACKET_LEN_I16+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_I16;
    buff[2] = channel;
    buff[3] = PACKET_DATA_I16;
    buff[4] = (value>>8)&0xff;
    buff[5] = value&0xff;

    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_I16);

    packet_send(cobs_buff,ret);
    
    return ret; 
}

uint8_t packet_wave_send_u32(uint8_t channel, uint32_t value)
{
    uint8_t buff[PACKET_LEN_U32] = {0};
    uint8_t cobs_buff[PACKET_LEN_U32+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_U32;
    buff[2] = channel;
    buff[3] = PACKET_DATA_U32;
    buff[4] = (value>>24)&0xff;
    buff[5] = (value>>16)&0xff;
    buff[6] = (value>>8)&0xff;
    buff[7] = value&0xff;

    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_U32);

    packet_send(cobs_buff,ret);
    
    return ret; 
}

uint8_t packet_wave_send_i32(uint8_t channel, int32_t value)
{
    uint8_t buff[PACKET_LEN_I32] = {0};
    uint8_t cobs_buff[PACKET_LEN_I32+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_I32;
    buff[2] = channel;
    buff[3] = PACKET_DATA_I32;
    buff[4] = (value>>24)&0xff;
    buff[5] = (value>>16)&0xff;
    buff[6] = (value>>8)&0xff;
    buff[7] = value&0xff;


    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_I32);

    packet_send(cobs_buff,ret);
    
    return ret; 
}

uint8_t packet_wave_send_float(uint8_t channel, float value)
{

    union {
        float f;
        uint32_t v;
    }f2u32;
    

    uint8_t buff[PACKET_LEN_FLOAT] = {0};
    uint8_t cobs_buff[PACKET_LEN_FLOAT+1] = {0};

    if (channel > CHANNEL_MAX) { 
        return 0;
    }

    f2u32.f = value;

    buff[0] = PACKET_PLOT;
    buff[1] = PACKET_LEN_FLOAT;
    buff[2] = channel;
    buff[3] = PACKET_DATA_FLOAT;
    buff[4] = (f2u32.v>>24)&0xff;
    buff[5] = (f2u32.v>>16)&0xff;
    buff[6] = (f2u32.v>>8)&0xff;
    buff[7] = f2u32.v&0xff;

    size_t ret = COBSEncode(cobs_buff,buff,PACKET_LEN_FLOAT);

    packet_send(cobs_buff,ret);
    
    return ret; 
}
