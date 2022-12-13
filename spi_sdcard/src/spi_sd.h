/**
 * @file spi_sd.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _SPI_SD_HEAD_H_
#define _SPI_SD_HEAD_H_

#include <stdint.h>

/**
 * @brief init sd card
 * 
 */
uint8_t spi_sd_init();

/**
 * @brief spi read sd
 * 
 * @param buff 
 * @param sector 
 */
uint8_t spi_sd_read(uint8_t *buff, uint32_t sector);

/**
 * @brief spi write sd
 * 
 * @param buff 
 * @param sector 
 */
uint8_t spi_sd_write(uint8_t *buff, uint32_t sector);

#endif //!_SPI_SD_HEAD_H_