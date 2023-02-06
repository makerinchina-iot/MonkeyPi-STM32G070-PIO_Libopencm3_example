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

#include "packet_wave.h"


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

void log(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);

    //log with segger rtt
    SEGGER_RTT_vprintf(0,fmt,&va); 

    //log with uart
    // printf(fmt,&va);

    va_end(va);
}

void send_plot_data_buff(uint8_t *buff, uint8_t len)
{

    //!!!use uart to plot 
    {
        // Serial.write(buff, len);
        // Serial.write(0x00);//the end

        // int i=0;
        // for(i=0; i<len; i++){
        //     usart_send_blocking(USART1,buff[i]);
        // }
        // usart_send_blocking(USART1,(char)(0x00));
        }


    //!!!use the segger rtt to plot
    {
        SEGGER_RTT_Write(0,buff,len);
        SEGGER_RTT_PutChar(0,0x00);
    }

}

int main(void)
{
    sys_clock_setup();

    //gpio setting
    rcc_periph_clock_enable(LED_GPIO_CLK);
    gpio_mode_setup(LED_GPIO_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,LED_GPIO_PIN);
    gpio_set_output_options(LED_GPIO_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_MED,LED_GPIO_PIN);

    uart_setup();

    // log("app log with segger rtt.\r\n");

    //plot the data
    packet_wave_send_fn_set(send_plot_data_buff);

    uint32_t cnt = 0;
    while (1)
    {
        //toggle gpio
        gpio_toggle(LED_GPIO_PORT,LED_GPIO_PIN);

        packet_wave_send_i8(0,cnt);

        packet_wave_send_u32(1,(uint32_t)(cnt)%100);
        
        packet_wave_send_float(2,100*sin(cnt*3.14/360.00));

        cnt++;

        //delay with 'nop' instruction
        delay_ms(20);
    }

    while(1){}

    return 0;
}