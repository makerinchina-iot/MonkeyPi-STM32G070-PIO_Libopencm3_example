/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief watch dog tutorial
 * @version 0.01
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/systick.h>

#define LED_1_RCC       RCC_GPIOC
#define LED_1_PORT      GPIOC
#define LED_1_PIN       GPIO12

static void systick_setup(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(64000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

volatile uint32_t tick_counter = 0;

void sys_tick_handler(void)
{
	tick_counter++;
}

uint32_t millis(void)
{
	return tick_counter;
}

int main(void)
{
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    rcc_periph_clock_enable(LED_1_RCC);

    gpio_mode_setup(LED_1_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,LED_1_PIN);

    systick_setup();

    //if reset led blink fast for 10 times
    int x = 0;
    uint32_t lastTime = millis();
    while(1){
        if( (millis() - lastTime) > 100) {
            lastTime = millis();
            gpio_toggle(LED_1_PORT,LED_1_PIN);

            if(x++ > 10){
                break;
            }
        }
    }

    //set wdt
    iwdg_set_period_ms(600); 

    iwdg_start();

    lastTime = millis();
    while(1){

        if( (millis() - lastTime) > 500) {
            lastTime = millis();

            gpio_toggle(LED_1_PORT,LED_1_PIN);

            iwdg_reset();
        }
        
    }
     
    while(1){}

    return 0;
}