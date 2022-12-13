/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief timer and systick tutorial
 * @version 0.01
 * @date 2022-09-10
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

volatile uint32_t tick_counter = 0;

static void sys_clock_setup(void)
{
	rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);
}

static void led_gpio_setup(void)
{
	/* Enable GPIO clock for leds. */
	rcc_periph_clock_enable(RCC_GPIOC);

	/* Enable led as output */
	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO12);
	gpio_set(GPIOC, GPIO12);
}

static void timer_setup(void)
{
    /* Enable TIM3 clock. */
	rcc_periph_clock_enable(RCC_TIM3);

	/* Enable TIM3 interrupt. */
	nvic_enable_irq(NVIC_TIM3_IRQ);

	/* Timer global mode:
	 * - No divider
	 * - Alignment edge
	 * - Direction up
	 */
	timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
		TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/*
	 * APB1 PRE = 1, TIMPCLK = PCLK
     * APB1 PRE != 1, TIMPCLK = PCLK * 2
	 */
	timer_set_prescaler(TIM3, (rcc_apb1_frequency/100000-1)); //100KHz

	/* Disable preload. */
	timer_disable_preload(TIM3);
	timer_continuous_mode(TIM3);

    /* Timer Period */
	timer_set_period(TIM3, 20000-1);	/* 100kHz /20_000 = 5 Hz */

	/* Counter enable. */
	timer_enable_counter(TIM3);

	timer_enable_irq(TIM3, TIM_DIER_UIE);
}

static void systick_setup(void)
{
	/* clock rate / 1000 to get 1mS interrupt rate */
	systick_set_reload(64000);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

uint32_t millis(void)
{
	return tick_counter;
}

int main(void)
{
    
    sys_clock_setup();

    led_gpio_setup();

	timer_setup();

	// systick_setup();

	uint32_t lastTime = millis();
    while(1){
		if( (millis() - lastTime) > 500) {
			lastTime = millis();

			gpio_toggle(GPIOC,GPIO12);
		}	
    }

    return 0;
}

void tim3_isr(void)
{
	if(timer_get_flag(TIM3, TIM_SR_UIF)) {
        /* Clear compare interrupt flag. */
		timer_clear_flag(TIM3, TIM_SR_UIF);

        gpio_toggle(GPIOC,GPIO12);
    }
}

void sys_tick_handler(void)
{
	tick_counter++;
}