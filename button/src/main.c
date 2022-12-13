/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief gpio input tutorial
 * @version 0.01
 * @date 2022-09-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/exti.h>

void button_setup(void)
{
  rcc_periph_clock_enable(RCC_GPIOC);

  nvic_enable_irq(NVIC_EXTI4_15_IRQ);

  gpio_mode_setup(GPIOC,
                GPIO_MODE_INPUT,
                GPIO_PUPD_NONE,
                GPIO11);

  exti_select_source(EXTI11, GPIOC);
  exti_set_trigger(EXTI11, EXTI_TRIGGER_FALLING);
  exti_enable_request(EXTI11);
}

int main(void)
{
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

    rcc_periph_clock_enable(RCC_GPIOC);

    //set gpio output for led
    gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);

    //set gpio input button
    // gpio_mode_setup(GPIOC, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO11);

    button_setup();

    while (1)
    {
        // if(gpio_get(GPIOC,GPIO11)){
            // gpio_toggle(GPIOC,GPIO12);
        // }    
    }    

    return 0;
}

/**
 * @brief EXTI4-15 Interrupt service routine.
 */
void exti4_15_isr(void)
{
  exti_reset_request(EXTI11);

  gpio_toggle(GPIOC, GPIO12);
}
