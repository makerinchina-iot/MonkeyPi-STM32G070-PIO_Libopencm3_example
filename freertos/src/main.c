/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief FreeRTOS led blink example
 * @version 0.01
 * @date 2022-09-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/systick.h>

#include "FreeRTOS.h"
#include "task.h"

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

static void led_task(void *args)
{
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO12);

    while (1)
    {
        gpio_toggle(GPIOC,GPIO12);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void)
{
    //system clock
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    systick_setup();

    xTaskCreate(led_task,"led task", 256, NULL,2,NULL);

    vTaskStartScheduler();
	
	while(1){}

    return 0;
}