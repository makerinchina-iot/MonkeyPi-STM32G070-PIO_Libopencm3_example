/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief PWM example
 * @version 0.01
 * @date 2022-09-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>

/**
 * @brief system clock setup
 * 
 */
static void sys_clock_setup(void)
{
    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);
}

/**
 * @brief gpio config 
 * 
 */
static void gpio_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOC);

    gpio_mode_setup(GPIOC,
                    GPIO_MODE_AF,
                    GPIO_PUPD_NONE,
                    GPIO12);

    gpio_set_output_options(GPIOC,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,GPIO12);

    //TIM14_CH1 , AF2
    gpio_set_af(GPIOC,GPIO_AF2,GPIO12);
}

/**
 * @brief pwm channel setup
 * 
 */
static void pwm_setup(void)
{
    rcc_periph_clock_enable(RCC_TIM14);

    /* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM14, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * APB1 PRE = 1, TIMPCLK = PCLK
     * APB1 PRE != 1, TIMPCLK = PCLK * 2
	 */
	timer_set_prescaler(TIM14, (rcc_apb1_frequency/100000-1)); //100KHz

	/* Disable preload. */
	timer_disable_preload(TIM14);
	timer_continuous_mode(TIM14);

    /* Timer Period */
	timer_set_period(TIM14, 20-1);	/* 100kHz /20 = 5 KHz */

    /* Set the initual output compare value for OC1. */
    timer_set_oc_mode(TIM14, TIM_OC1, TIM_OCM_PWM1);
    timer_set_oc_value(TIM14, TIM_OC1, 20*0.3); //duty = 0.3

    /* Enable output */
    timer_enable_oc_output(TIM14, TIM_OC1);
    timer_enable_counter(TIM14);
}

/**
 * @brief main function
 * 
 * @return int 
 */
int main(void)
{
    sys_clock_setup();

    gpio_setup();

    pwm_setup();

    int duty = 0;

    while(1){

        //from 0 - 100
        for(duty=0; duty <= 100; duty++){
            duty = duty + 1;
            timer_set_oc_value(TIM14,TIM_OC1, 20*duty/100);

            //delay some time
            for(int i=0; i<600000; i++){
                __asm__("nop");
            }
        }
        
        //from 100-0
        for(duty=100;duty>=0; duty--){
            duty = duty - 1;
            timer_set_oc_value(TIM14,TIM_OC1, 20*duty/100);

            //delay some time
            for(int i=0; i<600000; i++){
                __asm__("nop");
            }

        }
        
    }

    return 0;
}