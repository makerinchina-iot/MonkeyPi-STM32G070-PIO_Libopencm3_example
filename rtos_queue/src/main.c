/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief FreeRTOS queue tutotial
 * @version 0.01
 * @date 2022-09-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#include <string.h>

xQueueHandle uart_queue;

/**
 * @brief systick setup for rtos tick
 */
static void systick_setup(void)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(64*1000);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

/**
 * @brief system clock setup
 * 
 */
static void sys_clock_setup()
{
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);
}

/**
 * @brief uart setup
 * 
 */
static void uart_setup()
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

static void app_uart_task(void *param)
{
    char str[32] = {0};
    char i=0;

    printf(" app uart task\n");

    while(1){
        char c;
        if( xQueueReceive(uart_queue, &c, 5) == pdPASS) {

            //printf("recv:%c\n", c);

            if(c == '\n'){
                printf(" recv_str: %s\n", str);
                memset(str,0,32);
                i=0;
            }else{
                if(i<32){
                    str[i++] = c;
                }else{
                    i = 0;
                }
            }
        }
    }
}

int main(void)
{
    sys_clock_setup();

    systick_setup();

    uart_setup();

    //create uart print task
    xTaskCreate(app_uart_task, "uart task", 256, NULL, 2, NULL);

    //create queue
    uart_queue = xQueueCreate(16, sizeof(char));

    vTaskStartScheduler();

    while(1){}

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

        char c = usart_recv(USART1);

        xQueueSendFromISR(uart_queue, &c, NULL);
	}
}

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