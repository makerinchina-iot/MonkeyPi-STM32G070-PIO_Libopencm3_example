/**
 * @file sw_i2c_port.h
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef _SW_I2C_PORT_HEAD_H_
#define _SW_I2C_PORT_HEAD_H_

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define	SW_I2C_SCL_CLOCK	    RCC_GPIOB
#define SW_I2C_SCL_PORT   	    GPIOB
#define SW_I2C_SCL_PIN    	    GPIO13

#define	SW_I2C_SDA_CLOCK	    RCC_GPIOB
#define SW_I2C_SDA_PORT   	    GPIOB
#define SW_I2C_SDA_PIN    	    GPIO14

#define sw_i2c_scl_high()       gpio_set(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN)
#define sw_i2c_scl_low()        gpio_clear(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN)
#define sw_i2c_sda_high()       gpio_set(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN)
#define sw_i2c_sda_low()        gpio_clear(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN)

#define sw_i2c_sda_input()      gpio_mode_setup(SW_I2C_SDA_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, SW_I2C_SDA_PIN)
#define sw_i2c_sda_output()     gpio_mode_setup(SW_I2C_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_I2C_SDA_PIN)

// #define sw_i2c_delay()       delay_us(5)
#define sw_i2c_delay()          do{                                     \
                                    for (int i=0; i<58; i++) {          \
                                        __asm__ volatile ("nop");       \
                                    }                                   \
                                }while(0)
static bool sw_i2c_sda_get(void) 
{
    return (gpio_get(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN) != 0) ? true:false;
} 

static void sw_i2c_port_init()
{
    /* 打开GPIO时钟 */
    rcc_periph_clock_enable(SW_I2C_SCL_CLOCK);
    rcc_periph_clock_enable(SW_I2C_SDA_CLOCK);

    /* 禁用默认上拉，使SCL, SDA保持高阻状态, 设置为 OD 模式 */
    gpio_mode_setup(SW_I2C_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_I2C_SCL_PIN);
    gpio_mode_setup(SW_I2C_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SW_I2C_SDA_PIN);
    gpio_set_output_options(SW_I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, SW_I2C_SCL_PIN);
    gpio_set_output_options(SW_I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_25MHZ, SW_I2C_SDA_PIN);

    /* 空闲: 拉高SCL和SDA */
    gpio_set(SW_I2C_SCL_PORT, SW_I2C_SCL_PIN);
    gpio_set(SW_I2C_SDA_PORT, SW_I2C_SDA_PIN);
}

#endif //!_SW_I2C_PORT_HEAD_H_