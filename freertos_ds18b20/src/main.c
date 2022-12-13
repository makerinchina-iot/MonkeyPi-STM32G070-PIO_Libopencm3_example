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

#include "ds18b20.h"

#include "SEGGER_RTT.h"

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

static void led_task(void *args)
{

    

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);


    while (1)
    {
        gpio_toggle(GPIOB,GPIO4);
        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}


static void ds_task1(void *args)
{

    struct ds18b20 dev1_ds18b20; 
    dev1_ds18b20.pin = GPIO0;
    dev1_ds18b20.port = GPIOA;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO0);

    log("ds18b20 init1.\r\n");
    ds18b20_init(&dev1_ds18b20);


    while (1)
    {
        

        ds18b20_read_temp(&dev1_ds18b20);


        char buf[32] = {0};

        uint16_t v1 = dev1_ds18b20.temp.value;


        log("temp1: %d.%d\r\n",v1/10, v1%10);

        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}


static void ds_task2(void *args)
{



    struct ds18b20 dev2_ds18b20;
    dev2_ds18b20.pin = GPIO1;
    dev2_ds18b20.port = GPIOA;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO1);

    log("ds18b20 init2.\r\n");

    ds18b20_init(&dev2_ds18b20);


    while (1)
    {

        ds18b20_read_temp(&dev2_ds18b20);


        char buf[32] = {0};


        uint16_t v2 = dev2_ds18b20.temp.value;



        log("temp2: %d.%d\r\n",v2/10,v2%10);

        vTaskDelay(pdMS_TO_TICKS(1200));

    }
}


static void ds_task3(void *args)
{



        struct ds18b20 dev3_ds18b20;
    dev3_ds18b20.pin = GPIO2;
    dev3_ds18b20.port = GPIOA;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO2);

    log("ds18b20 init3.\r\n");

    ds18b20_init(&dev3_ds18b20);


    while (1)
    {

        ds18b20_read_temp(&dev3_ds18b20);


        char buf[32] = {0};


        uint16_t v3 = dev3_ds18b20.temp.value;


        log("temp3: %d.%d\r\n",v3/10, v3%10);

        vTaskDelay(pdMS_TO_TICKS(1200));
    }
}


static void ds_task4(void *args)
{

    struct ds18b20 dev4_ds18b20;
    dev4_ds18b20.pin = GPIO3;
    dev4_ds18b20.port = GPIOA;



    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO3);

    log("ds18b20 init4.\r\n");

    ds18b20_init(&dev4_ds18b20);

    while (1)
    {

        ds18b20_read_temp(&dev4_ds18b20);

        char buf[32] = {0};


        uint16_t v4 = dev4_ds18b20.temp.value;

        log("temp4: %d.%d\r\n",v4/10, v4%10);

        vTaskDelay(pdMS_TO_TICKS(1200));



    }
}

void ds_test()
{

    rcc_periph_clock_enable(RCC_GPIOB);
    gpio_mode_setup(GPIOB,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);
    
    struct ds18b20 dev1_ds18b20; 
    dev1_ds18b20.pin = GPIO0;
    dev1_ds18b20.port = GPIOA;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO0);



    struct ds18b20 dev2_ds18b20;
    dev2_ds18b20.pin = GPIO1;
    dev2_ds18b20.port = GPIOA;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO1);



    struct ds18b20 dev3_ds18b20;
    dev3_ds18b20.pin = GPIO4;
    dev3_ds18b20.port = GPIOA;

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);



    struct ds18b20 dev4_ds18b20;
    dev4_ds18b20.pin = GPIO4;
    dev4_ds18b20.port = GPIOC;



    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);

    log("ds18b20 init1.\r\n");
    ds18b20_init(&dev1_ds18b20);

    log("ds18b20 init2.\r\n");
    ds18b20_init(&dev2_ds18b20);

    log("ds18b20 init3.\r\n");
    ds18b20_init(&dev3_ds18b20);

    log("ds18b20 init4.\r\n");
    ds18b20_init(&dev4_ds18b20);

    while (1)
    {

        // log("get dev1:%d\r\n", dev1_ds18b20.pin);

        ds18b20_read_temp(&dev1_ds18b20);

        // log("get dev2:%d\r\n", dev2_ds18b20.pin);
        ds18b20_read_temp(&dev2_ds18b20);
        
        ds18b20_read_temp(&dev3_ds18b20);
        ds18b20_read_temp(&dev4_ds18b20);

        uint16_t v1 = dev1_ds18b20.temp.value;
        uint16_t v2 = dev2_ds18b20.temp.value;
        uint16_t v3 = dev3_ds18b20.temp.value;
        uint16_t v4 = dev4_ds18b20.temp.value;

        // log("temp1: %d.%d\r\n",v1/10, v1%10);
        // log("temp2: %d.%d\r\n",v2/10, v2%10);
        // log("temp3: %d.%d\r\n",v3/10, v3%10);
        // log("temp4: %d.%d\r\n",v4/10, v4%10);

        log("\r\ntemp: %d.%d    %d.%d   %d.%d   %d.%d\r\n", v1/10,v1%10,v2/10,v2%10,v3/10,v3%10,v4/10,v4%10);

        mdelay(100);

        gpio_toggle(GPIOB,GPIO4);

    }

}

int main(void)
{
    //system clock
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    systick_setup();

    delay_init();

    // xTaskCreate(led_task,"led task", 256, NULL,2,NULL);

    // xTaskCreate(ds_task1,"ds task1", 256, NULL,2,NULL);
    // xTaskCreate(ds_task2,"ds task2", 256, NULL,2,NULL);

    // xTaskCreate(ds_task3,"ds task3", 256, NULL,2,NULL);
    // xTaskCreate(ds_task4,"ds task4", 256, NULL,2,NULL);

    // vTaskStartScheduler();

    ds_test();
	
	while(1){}

    return 0;
}