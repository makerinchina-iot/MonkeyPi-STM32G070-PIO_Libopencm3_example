/**
 * @file sw_i2c.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _SW_I2C_HEAD_H_
#define _SW_I2C_HEAD_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/**
 * @brief init i2c
 * 
 */
void sw_i2c_init();

/**
 * @brief i2c transfer data
 * 
 * @param dev_addr  slave address
 * @param tx_buffer buff send data      
 * @param tx_size   count to send 
 * @param rx_buffer buff receive data 
 * @param rx_size   count to receive 
 * @return true      
 * @return false 
 */
bool sw_i2c_transfer(uint8_t dev_addr, uint8_t *tx_buffer,uint16_t tx_size,uint8_t *rx_buffer,uint16_t rx_size);

/**
 * @brief scan device on i2c bus
 * 
 * @param from_addr scan from 
 * @param to_addr   scan to 
 * @param callback  the result scan cb: address-the scan address, result 0 indicate found the address of device
 */
void scan_i2c_bus(uint8_t from_addr, uint8_t to_addr, void(*callback)(uint8_t address, uint8_t result));

#endif //!_SW_I2C_HEAD_H_