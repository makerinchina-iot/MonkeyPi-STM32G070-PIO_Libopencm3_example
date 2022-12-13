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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "sw_i2c.h"

volatile uint8_t recv_index = 0;
volatile uint8_t send_index = 0;

#define BUFF_SIZE 64
uint8_t recv_buff[BUFF_SIZE] = {0};

/*
 * Called by libc stdio fwrite functions
 */
int
_write(int fd, char *ptr, int len)
{
	int i = 0;

	/*
	 * Write "len" of char from "ptr" to file id "fd"
	 * Return number of char written.
	 *
	 * Only work for STDOUT, STDIN, and STDERR
	 */
	if (fd > 2) {
		return -1;
	}
	while (*ptr && (i < len)) {
		usart_send_blocking(USART1, *ptr);
		if (*ptr == '\n') {
			usart_send_blocking(USART1, '\r');
		}
		i++;
		ptr++;
	}
	return i;
}

void sys_clock_setup(void)
{
    //system clock

    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

}

void uart_setup(void)
{

    //uart pin
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(GPIOB,GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO6|GPIO7);
    gpio_set_af(GPIOB,GPIO_AF0,GPIO6|GPIO7);

    usart_set_baudrate(USART1,115200);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1,USART_MODE_TX_RX);

    //uart isr
    nvic_enable_irq(NVIC_USART1_IRQ);

    usart_enable(USART1);

    usart_enable_rx_interrupt(USART1);
}

static inline void delay_ms(uint32_t n)
{
	for(uint32_t i=0; i<n; i++){
		for(int x=0;x<12806;x++){
			__asm__("nop");
		}
	}
}

void gxht30_sample(float *temp, float *humi)
{
    uint8_t rd_buff[6] = {0};
    uint8_t cmd[2] = {0x2c, 0x06};

    uint8_t dev_addr = 0x44;

    //send read cmd
    sw_i2c_transfer(dev_addr, cmd, 2, 0, 0);

    delay_ms(10);

    //receive data
    sw_i2c_transfer(dev_addr, 0,0, rd_buff, 6);

    uint16_t temp_int = (uint16_t)((rd_buff[0] << 8)|(rd_buff[1]));
    uint16_t humi_int = (uint16_t)((rd_buff[3] << 8)|(rd_buff[4]));

    *temp =  -45 + (float)(175*temp_int/65535.0000);
    *humi = 100 * (float)(humi_int /65535.0000);
}

void scan_i2c_cb( uint8_t addr, uint8_t result )
{

    if(result == 1){
        printf("  scan addr[7bit]: 0x%x  found!\r\n",addr);
    }else{
    //    printf("scan addr:   %x not found\r\n",addr); //not found
    }

}

int main(void)
{
    sys_clock_setup();

    uart_setup();

    printf("init i2c bus\r\n");

    sw_i2c_init();

    printf("scan device on i2c bus...\r\n");

    scan_i2c_bus(0x02,0xfe, scan_i2c_cb);

    while(1){
        float temp,humi;
        gxht30_sample(&temp, &humi);

        printf("T&H : %.2f, %.2f\r\n", temp, humi);

        delay_ms(1000);
    }

    while(1){/*do nothing*/}///!!!not go here.
    return 0;
}