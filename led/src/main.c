/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#define     LED_GPIO_CLK     RCC_GPIOC
#define     LED_GPIO_PORT    GPIOC
#define     LED_GPIO_PIN     GPIO12

int main(void)
{
    
    //system clock
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    //gpio setting
    rcc_periph_clock_enable(LED_GPIO_CLK);
    gpio_mode_setup(LED_GPIO_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,LED_GPIO_PIN);
    gpio_set_output_options(LED_GPIO_PORT,GPIO_OTYPE_OD,GPIO_OSPEED_MED,LED_GPIO_PIN);

    while (1)
    {
        //toggle gpio
        gpio_toggle(LED_GPIO_PORT,LED_GPIO_PIN);

        //delay with 'nop' instruction
        for(int i=0; i<2000000; i++){
            __asm__("nop;");
        }
    }

    return 0;
}
