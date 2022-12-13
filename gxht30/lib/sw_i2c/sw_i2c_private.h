/**
 * @file sw_i2c_private.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _SW_I2C_PRIVATE_HEAD_H_
#define _SW_I2C_PRIVATE_HEAD_H_

#include "sw_i2c_port.h"

static void i2c_start(void);
static void i2c_stop(void);
static bool i2c_wait_ack(void);
static void i2c_send_ack(void);
static void i2c_send_nack(void);
static void i2c_send_byte(uint8_t data);
static uint8_t i2c_recv_byte(bool ack);

/**
 * @brief I2C总线启动信号
 */
static void i2c_start(void)
{
    /* 当SCL高电平时，SDA出现一个下跳沿表示I2C总线启动信号 */
    sw_i2c_sda_high();
    sw_i2c_scl_high();
    sw_i2c_delay();
    sw_i2c_sda_low();
    sw_i2c_delay();
    sw_i2c_scl_low();
    sw_i2c_delay();
}

/**
 * @brief I2C总线停止信号
 */
static void i2c_stop(void)
{
    /* 当SCL高电平时，SDA出现一个上跳沿表示I2C总线停止信号 */
    sw_i2c_sda_low();
    sw_i2c_delay();
    sw_i2c_scl_high();
    sw_i2c_delay();
    sw_i2c_sda_high();
}

/**
 * @brief 向I2C总线设备发送1个字节
 * @param data 等待发送的字节
 */
static void i2c_send_byte(uint8_t data)
{
    uint8_t i;

    /* 先发送字节的高位bit7 */
    for (i = 0; i < 8; i++) {
        sw_i2c_delay();
        sw_i2c_scl_low();

        if (data & 0x80) {
            sw_i2c_sda_high();
        } else {
            sw_i2c_sda_low();
        }

        sw_i2c_delay();
        sw_i2c_scl_high();

        data <<= 1;	/* 左移一个bit */
    }
}

/**
 * @brief 产生一个时钟，并读取器件的ACK应答信号
 * @return 返回true表示正确应答，false表示无器件响应
 */
static bool i2c_wait_ack(void)
{
	bool res;

	sw_i2c_delay();
	sw_i2c_scl_low();

	sw_i2c_sda_input();

    sw_i2c_delay();
    sw_i2c_scl_high();	/* 驱动SCL = 1, 此时器件会返回ACK应答 */
    sw_i2c_delay();
    if (sw_i2c_sda_get() == false) { /* 读取SDA口线状态 */
        res = true;
    } else {
        res = false;
    }
    sw_i2c_scl_low();
    sw_i2c_sda_high();	/* 释放SDA总线 */
	sw_i2c_sda_output();
    sw_i2c_delay();

    return res;
}

/**
 * @brief 产生一个ACK信号
 */
static void i2c_send_ack(void)
{
    sw_i2c_sda_low();	/* CPU驱动SDA = 0 */
    sw_i2c_delay();
    sw_i2c_scl_high();	/* CPU产生1个时钟 */
    sw_i2c_delay();
    sw_i2c_scl_low();
    sw_i2c_delay();
    sw_i2c_sda_high();	/* CPU释放SDA总线 */
}

/**
 * @brief CPU产生1个NACK信号
 */
static void i2c_send_nack(void)
{
    sw_i2c_sda_high();	/* CPU驱动SDA = 1 */
    sw_i2c_delay();
    sw_i2c_scl_high();	/* CPU产生1个时钟 */
    sw_i2c_delay();
    sw_i2c_scl_low();
    sw_i2c_delay();
}

/**
 * @brief CPU从I2C总线设备读取8bit数据
 * 读1个字节，ack=1时，发送ACK，ack=0，发送nACK
 * @return
 */
static uint8_t i2c_recv_byte(bool ack)
{
    uint8_t i;
    uint8_t value;

    /* 读到第1个bit为数据的bit7 */
    value = 0;
    for (i = 0; i < 8; i++) {
        value <<= 1;
        sw_i2c_scl_high();
        sw_i2c_delay();
        if (sw_i2c_sda_get()==true) {
            value++;
        }
        sw_i2c_scl_low();
        sw_i2c_delay();
    }

    if (ack) {
        i2c_send_ack(); //发送ACK
    } else {
        i2c_send_nack();//发送nACK
    }

    return value;
}

#endif //!_SW_I2C_PRIVATE_HEAD_H_