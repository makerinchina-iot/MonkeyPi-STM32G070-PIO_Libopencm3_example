/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>
#include <stdarg.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "SEGGER_RTT.h"

#include "FreeRTOS.h"
#include "task.h"

#include "mb.h"
#include "mbport.h"

void sys_clock_setup(void)
{
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);
}

static void systick_setup(void)
{
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_set_reload(64*1000);

	systick_interrupt_enable();

	/* Start counting. */
	systick_counter_enable();
}

void log(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    SEGGER_RTT_vprintf(0,fmt,&va);
    SEGGER_RTT_printf(0, "\r\n");//add new line
    va_end(va);
}

static void task_modbus_handle(void *param)
{

    eMBErrorCode    eStatus;

    log("  task modbus start.");

    eStatus = eMBInit( MB_RTU, 0x01, 0, 9600, MB_PAR_NONE );

    /* Enable the Modbus Protocol Stack. */
    eStatus = eMBEnable();

	(void)eStatus;

    for( ;; ) {
        ( void )eMBPoll();
        vTaskDelay(pdMS_TO_TICKS(10));
    }

}

int main(void)
{
    sys_clock_setup();

    xTaskCreate(task_modbus_handle, "modbus", 512, NULL, 4, NULL);

    vTaskStartScheduler();

    while(1){/*do nothing*/}///!!!not go here.
    return 0;
}