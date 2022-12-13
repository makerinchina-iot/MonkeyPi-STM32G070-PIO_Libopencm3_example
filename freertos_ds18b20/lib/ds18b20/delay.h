

#ifndef __DELAY_H__
#define __DELAY_H__

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/timer.h>

void delay_init(void);
void delay_us(uint32_t us);

#endif