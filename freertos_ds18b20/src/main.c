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
#include "queue.h"
#include "task.h"

// #include "ds18b20.h"

#include "ow_ds18b20.h"

#include "SEGGER_RTT.h"

typedef struct _temp_t{
    uint32_t index;
    uint16_t value;
}temp_t;

xQueueHandle x_temp_queue;

#define TASK_WAIT_TIME      200
#define QUEUE_SEND_TIMEOUT  100

void log(const char* fmt, ...)
{
    va_list va;
    va_start(va, fmt);
    SEGGER_RTT_vprintf(0,fmt,&va);
    va_end(va);
}

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

static uint32_t counter = 0;
static uint16_t buff[11] = {0};

static void led_task(void *args)
{

    

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);

    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO0);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO1);

    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO2);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO3);

    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);
    gpio_mode_setup(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);

    gpio_mode_setup(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO5);

    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO1);
    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO2);
    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO10);
    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO11);

    ow_t obj_list[11];

    obj_list[0].port = GPIOA;
    obj_list[0].pin = GPIO0;

        obj_list[1].port = GPIOA;
    obj_list[1].pin = GPIO1;

        obj_list[2].port = GPIOA;
    obj_list[2].pin = GPIO2;

        obj_list[3].port = GPIOA;
    obj_list[3].pin = GPIO3;

        obj_list[4].port = GPIOA;
    obj_list[4].pin = GPIO4;

        obj_list[5].port = GPIOC;
    obj_list[5].pin = GPIO4;

            obj_list[6].port = GPIOC;
    obj_list[6].pin = GPIO5;

            obj_list[7].port = GPIOB;
    obj_list[7].pin = GPIO1;

            obj_list[8].port = GPIOB;
    obj_list[8].pin = GPIO2;

            obj_list[9].port = GPIOB;
    obj_list[9].pin = GPIO10;

            obj_list[10].port = GPIOB;
    obj_list[10].pin = GPIO11;



    ow_ds18b20_init(obj_list, 11);

    // delay_us(1000000);

    ow_ds18b20_conv_temp(obj_list,11);

        log("---init.\r\n");

    vTaskDelay(1000);

    log("---init done\r\n");


    while (1)
    {
        gpio_toggle(GPIOB,GPIO4);

        // ow_ds18b20_conv_temp(obj_list,11);

        //should dealy > 900ms
        // delay_us(900*1000);
        // vTaskDelay(pdMS_TO_TICKS(800));

        ow_ds18b20_get_temp(obj_list, 11);

        ow_ds18b20_conv_temp(obj_list,11);

        vTaskDelay(pdMS_TO_TICKS(800));

        log("\r\n temp:");
        for(int i=0; i<11; i++){
            log("%d.%d  ",obj_list[i].temp/10, obj_list[i].temp%10);
        }
        log("\r\n");

        // vTaskDelay(pdMS_TO_TICKS(500));
    }
}

int main(void)
{
    //system clock
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    systick_setup();

    delay_init();

    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    xTaskCreate(led_task,"led task", 256, NULL,2,NULL);

    vTaskStartScheduler();

	while(1){}

    return 0;
}