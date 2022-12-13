/*
 * FreeModbus Libary: BARE Port
 * Copyright (C) 2006 Christian Walter <wolti@sil.at>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 * File: $Id$
 */


#include "port.h"

#include "FreeRTOS.h"
#include "queue.h"

#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

/* ----------------------- Modbus includes ----------------------------------*/
#include "mb.h"
#include "mbport.h"

/* ----------------------- static functions ---------------------------------*/

xQueueHandle uart_queue;

#define RS485_1_CLOCK		RCC_GPIOB
#define RS485_1_EN_PORT		GPIOB
#define RS485_1_EN_PIN		GPIO8

static void rs485_delay(int n)
{
    while (--n) {
        __asm__ volatile ("nop");
    }
}

static inline void rs485_1_rx_mode(void)
{
    gpio_clear(RS485_1_EN_PORT, RS485_1_EN_PIN);
}

static inline void rs485_1_tx_mode(void)
{
    gpio_set(RS485_1_EN_PORT, RS485_1_EN_PIN);
}

static inline void rs485_gpio_init(void)
{
	rcc_periph_clock_enable(RS485_1_CLOCK);
	gpio_mode_setup(RS485_1_EN_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, RS485_1_EN_PIN);

	rs485_1_rx_mode();
}

/* ----------------------- Start implementation -----------------------------*/
void
vMBPortSerialEnable( BOOL xRxEnable, BOOL xTxEnable )
{
    /* If xRXEnable enable serial receive interrupts. If xTxENable enable
     * transmitter empty interrupts.
     */
    if (xRxEnable) {
        rs485_delay(10000);
        rs485_1_rx_mode();
        rs485_delay(10000);
		usart_enable_rx_interrupt(USART1);
    }
    else {
		usart_disable_rx_interrupt(USART1);
    }
    
    if (xTxEnable) {
        rs485_delay(10000);
        rs485_1_tx_mode();
        rs485_delay(10000);
		usart_enable_tx_interrupt(USART1);
    }
    else {
		usart_disable_tx_interrupt(USART1);

    }
}

BOOL
xMBPortSerialInit( UCHAR ucPORT, ULONG ulBaudRate, UCHAR ucDataBits, eMBParity eParity )
{
	nvic_enable_irq(NVIC_USART1_IRQ);

	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO6 | GPIO7);
	gpio_set_af(GPIOB, GPIO_AF0, GPIO6 | GPIO7);

	rcc_periph_clock_enable(RCC_USART1);

	/* Set up USART/UART parameters using the libopencm3 helper functions */
	usart_set_baudrate(USART1, ulBaudRate);
	usart_set_databits(USART1, ucDataBits);
	usart_set_stopbits(USART1, USART_STOPBITS_1);
	usart_set_mode(USART1, USART_MODE_TX_RX);

    switch (eParity) {
        case MB_PAR_ODD:
			usart_set_parity(USART1, USART_PARITY_ODD);
            break;
        case MB_PAR_EVEN:
			usart_set_parity(USART1, USART_PARITY_EVEN);
            break;
        default:
			usart_set_parity(USART1, USART_PARITY_NONE);
            break;
    }

	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

	usart_enable(USART1);

    rs485_gpio_init();

    return TRUE;
}

BOOL
xMBPortSerialPutByte( CHAR ucByte )
{

    usart_send_blocking(USART1, (uint16_t) ucByte);    
    
    return TRUE;
}

BOOL
xMBPortSerialGetByte( CHAR * pucByte )
{
	*pucByte = usart_recv(USART1);

    return TRUE;
}


uint32_t uart1_isr, uart1_icr;

void usart1_isr(void)
{

	/* Check if we were called because of RXNE. */
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_RXNE) != 0)) {

		/* Retrieve the data from the peripheral. */
        // usart_recv(USART1);

		pxMBFrameCBByteReceived();

	}


	/* Check if we were called because of TXE. */
	if (((USART_CR1(USART1) & USART_CR1_TXEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_TXE) != 0)) {

		/* Put data into the transmit register. */
		//usart_send(USART1, data);

		pxMBFrameCBTransmitterEmpty();

	}

}
