/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-25
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "sw_i2c.h"

#include "SEGGER_RTT.h"

void sys_clock_setup(void)
{
    //system clock
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

}

static inline void delay_ms(uint32_t n)
{
	for(uint32_t i=0; i<n; i++){
		for(int x=0;x<12806;x++){
			__asm__("nop");
		}
	}
}

#define M24CXX_DEVICE_ADDR     0x50

static inline bool m24cxx_read_one_byte(uint8_t addr, uint8_t *ptr)
{
    return sw_i2c_transfer(M24CXX_DEVICE_ADDR, &addr, 1, ptr, 1);
}

static inline bool m24cxx_write_one_byte(uint8_t addr, uint8_t data)
{
    uint8_t buffer[2];

    buffer[0] = addr;
    buffer[1] = data;
    return sw_i2c_transfer(M24CXX_DEVICE_ADDR, buffer, 2, 0, 0);
}

void log(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    SEGGER_RTT_vprintf(0,fmt,&va);
    va_end(va);
}

int main(void)
{
    sys_clock_setup();

    log("init sw i2c.\r\n");

    sw_i2c_init();

    log("eeprom read testing.\r\n");

    uint8_t addr = 0x01;
    uint8_t read_data=0;
    uint8_t write_data = 0x88;

    #if 1
        m24cxx_read_one_byte(addr,&read_data);

        if(read_data == write_data){
            log("eeprom write ok\r\n");
        }else{
            log("ERR:EEPROM write failed.\r\n");
        }
    
    #endif

    log("eeprom write.\r\n");

    m24cxx_write_one_byte(addr, write_data);

    delay_ms(5);//wait write done

    while(1){/*do nothing*/}///!!!not go here.
    return 0;
}

