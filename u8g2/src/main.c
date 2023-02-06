/**
 * @file main.cpp
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2023-01-14
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "SEGGER_RTT.h"

#define     LED_GPIO_CLK     RCC_GPIOC
#define     LED_GPIO_PORT    GPIOC
#define     LED_GPIO_PIN     GPIO12

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/i2c.h>

#include "u8x8.h"

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

/* Initialize I2C1 interface */
static void i2c1_setup(void) 
{
	/* Enable GPIOB and I2C1 clocks */
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_I2C1);

	/* Set alternate functions for SCL and SDA pins of I2C1 */
    gpio_mode_setup(GPIOB,GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO8|GPIO9);
    gpio_set_af(GPIOB,GPIO_AF6,GPIO8|GPIO9);

	/* Disable the I2C peripheral before configuration */
	i2c_peripheral_disable(I2C1);

    i2c_set_speed(I2C1,i2c_speed_fm_400k,8);

	/* And go */
	i2c_peripheral_enable(I2C1);
}

static uint8_t u8x8_gpio_and_delay_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) 
{
	switch(msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		i2c1_setup();  /* Init I2C communication */
		break;

	default:
		u8x8_SetGPIOResult(u8x8, 1);
		break;
	}

	return 1;
}

/* I2C hardware transfer based on u8x8_byte.c implementation */
static uint8_t u8x8_byte_hw_i2c_stm32(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int, void *arg_ptr) 
{
	static uint8_t buffer[32];   /* u8g2/u8x8 will never send more than 32 bytes */
	static uint8_t buf_idx;
	uint8_t *data;

	switch(msg) {
	case U8X8_MSG_BYTE_SEND:
		data = (uint8_t *)arg_ptr;
		while(arg_int > 0) {
			buffer[buf_idx++] = *data;
			data++;
			arg_int--;
		}
		break;
	case U8X8_MSG_BYTE_INIT:
		break;
	case U8X8_MSG_BYTE_SET_DC:
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		buf_idx = 0;
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		i2c_transfer7(I2C1, 0x3C, buffer, buf_idx, NULL, 0);
		break;
	default:
		return 0;
	}
	return 1;
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

    //gpio setting
    rcc_periph_clock_enable(LED_GPIO_CLK);
    gpio_mode_setup(LED_GPIO_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,LED_GPIO_PIN);
    gpio_set_output_options(LED_GPIO_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_MED,LED_GPIO_PIN);

    u8x8_t u8x8_i, *u8x8 = &u8x8_i;

    log("app log with segger rtt.\r\n");

    u8x8_Setup(u8x8, u8x8_d_ssd1306_128x64_noname, u8x8_cad_ssd13xx_fast_i2c, u8x8_byte_hw_i2c_stm32, u8x8_gpio_and_delay_stm32);

	u8x8_InitDisplay(u8x8);
	u8x8_SetPowerSave(u8x8,0);
	u8x8_SetFont(u8x8, u8x8_font_7x14B_1x2_f);

	u8x8_ClearDisplay(u8x8);
	u8x8_DrawString(u8x8, 1,1, "MonkeyPi");
	// u8x8_Draw2x2Glyph(u8x8, 0,0, 'M');
	u8x8_SetInverseFont(u8x8, 1);
	u8x8_DrawString(u8x8, 0,5, "MakerInChina.cn");
	u8x8_SetInverseFont(u8x8, 0);

	u8x8_SetFont(u8x8, u8x8_font_open_iconic_embedded_2x2);
	u8x8_DrawGlyph(u8x8, 11,1, 74); /* Bluetooth */

    while (1)
    {
        //toggle gpio
        gpio_toggle(LED_GPIO_PORT,LED_GPIO_PIN);

        log("led blink counter\r\n");

        //delay with 'nop' instruction
        delay_ms(200);
    }

    while(1){}

    return 0;
}