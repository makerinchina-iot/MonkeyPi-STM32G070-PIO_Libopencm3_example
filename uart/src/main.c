/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief uart isr usage tutorial
 * @version 0.01
 * @date 2022-09-08
 * 
 * @copyright Copyright (c) 2022
 * 
 */


#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>


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

int main(void)
{
    
    //system clock
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

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

    char buff[32] = "hello, makerinchina.cn\n";
    for(int i=0; i<strlen(buff); i++){
        usart_send_blocking(USART1, buff[i]);
    }

    printf(" printf function testing.\n");

    while (1)
    {

        if(recv_index != send_index){ 

            if(send_index < BUFF_SIZE){
                usart_send_blocking(USART1, recv_buff[send_index++]);
            }else{
                send_index = 0;
            }
        }

    }

    return 0;
}

/**
 * @brief uart1 isr function
 * 
 */
void usart1_isr(void)
{
    //receive interrupt
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_RXNE) != 0)) {

        if(recv_index < BUFF_SIZE){
            recv_buff[recv_index++] = usart_recv(USART1);
        }else{
            recv_index = 0;
        }
	}
}