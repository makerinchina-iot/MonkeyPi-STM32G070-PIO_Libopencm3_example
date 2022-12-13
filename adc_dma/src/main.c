/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-26
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <stdio.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/dmamux.h>


volatile uint8_t recv_index = 0;
volatile uint8_t send_index = 0;

#define BUFF_SIZE 64
uint8_t recv_buff[BUFF_SIZE] = {0};

/*
 * Called by libc stdio fwrite functions
 */
int
_write(int fd, char *ptr, int len)
{
	int i = 0;

	/*
	 * Write "len" of char from "ptr" to file id "fd"
	 * Return number of char written.
	 *
	 * Only work for STDOUT, STDIN, and STDERR
	 */
	if (fd > 2) {
		return -1;
	}
	while (*ptr && (i < len)) {
		usart_send_blocking(USART1, *ptr);
		if (*ptr == '\n') {
			usart_send_blocking(USART1, '\r');
		}
		i++;
		ptr++;
	}
	return i;
}

void sys_clock_setup(void)
{
    //system clock

    rcc_clock_setup(&rcc_clock_config[RCC_CLOCK_CONFIG_HSI_PLL_64MHZ]);

}

void uart_setup(void)
{

    //uart pin
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_GPIOB);

    gpio_mode_setup(GPIOB,GPIO_MODE_AF,GPIO_PUPD_NONE,GPIO6|GPIO7);
    gpio_set_af(GPIOB,GPIO_AF0,GPIO6|GPIO7);

    usart_set_baudrate(USART1,115200);
    usart_set_databits(USART1,8);
    usart_set_stopbits(USART1,USART_STOPBITS_1);
    usart_set_parity(USART1,USART_PARITY_NONE);
    usart_set_flow_control(USART1,USART_FLOWCONTROL_NONE);
    usart_set_mode(USART1,USART_MODE_TX_RX);

    //uart isr
    nvic_enable_irq(NVIC_USART1_IRQ);

    usart_enable(USART1);

    usart_enable_rx_interrupt(USART1);
}

static inline void delay_ms(uint32_t n)
{
	for(uint32_t i=0; i<n; i++){
		for(int x=0;x<12806;x++){
			__asm__("nop");
		}
	}
}

////////////////////////////////////////////////////////////////////
//ADC setup

#define ADC_CHAN_CNT        4
#define ADC_FILETER_SIZE    32

int16_t adc_values[ADC_FILETER_SIZE*ADC_CHAN_CNT];

static void adc_setup(void)
{
    rcc_periph_clock_enable(RCC_ADC);
    rcc_periph_clock_enable(RCC_GPIOA);

    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO2);
    gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO3);

    adc_power_off(ADC1);
    adc_set_clk_prescale(ADC1, ADC_CCR_PRESC_DIV2);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPTIME_160DOT5);

    uint8_t channel_array[16] = {0};
    channel_array[0] = 0;
    channel_array[1] = 1;
    channel_array[2] = 2;
    channel_array[3] = 3;
    adc_set_regular_sequence(ADC1, ADC_CHAN_CNT, channel_array);
    adc_enable_dma_circular_mode(ADC1);
    adc_set_resolution(ADC1, ADC_CFGR1_RES_12_BIT);
    adc_power_on(ADC1);

    /* Wait for ADC starting up. */
    delay_ms(10); 
}


static void dma_setup(void *data, int size)
{
    dma_channel_reset(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_address(DMA1, DMA_CHANNEL1, (uint32_t)&ADC_DR(ADC1));
    dma_set_memory_address(DMA1, DMA_CHANNEL1, (uint32_t)data);
    dma_set_number_of_data(DMA1, DMA_CHANNEL1, size);
    dma_set_read_from_peripheral(DMA1, DMA_CHANNEL1);
    dma_enable_memory_increment_mode(DMA1, DMA_CHANNEL1);
    dma_set_peripheral_size(DMA1, DMA_CHANNEL1, DMA_CCR_PSIZE_16BIT);
    dma_set_memory_size(DMA1, DMA_CHANNEL1, DMA_CCR_MSIZE_16BIT);
    dma_enable_circular_mode(DMA1, DMA_CHANNEL1);
    dma_enable_transfer_complete_interrupt(DMA1, DMA_CHANNEL1);
    dma_enable_channel(DMA1, DMA_CHANNEL1);

    dmamux_reset_dma_channel(DMAMUX1, DMA_CHANNEL1);
    dmamux_set_dma_channel_request(DMAMUX1, DMA_CHANNEL1, DMAMUX_CxCR_DMAREQ_ID_ADC);
}

void tim3_setup(void)
{
    /* Enable TIM3 clock. */
    rcc_periph_clock_enable(RCC_TIM3);

    /* Enable TIM3 interrupt. */
    nvic_enable_irq(NVIC_TIM3_IRQ);

    /* Reset TIM3 peripheral to defaults. */
    rcc_periph_reset_pulse(RST_TIM3);

    /* Timer global mode:
     * - No divider
     * - Alignment edge
     * - Direction up
     * (These are actually default values after reset above, so this call
     * is strictly unnecessary, but demos the api for alternative settings)
     */
    timer_set_mode(TIM3, TIM_CR1_CKD_CK_INT,
                   TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

    /*
     * Please take note that the clock source for STM32 timers
     * might not be the raw APB1/APB2 clocks.  In various conditions they
     * are doubled.  See the Reference Manual for full details!
     * In our case, TIM3 on APB1 is running at double frequency, so this
     * sets the prescaler to have the timer run at 5kHz
     */
    timer_set_prescaler(TIM3, 64-1);

    /* Disable preload. */
    timer_disable_preload(TIM3);
    timer_continuous_mode(TIM3);

    timer_set_period(TIM3, 20000-1); //100Hz

    timer_set_master_mode(TIM3, TIM_CR2_MMS_UPDATE);

    timer_enable_irq(TIM3, TIM_DIER_UIE);
}

void tim3_enable_counter(bool en)
{
    if(en){
        timer_enable_counter(TIM3);
    }else{
        timer_disable_counter(TIM3);
    }
}

void dma1_channel1_isr(void)
{

    if ((DMA1_ISR &DMA_ISR_TCIF1) != 0) {
        DMA1_IFCR |= DMA_IFCR_CTCIF1;
    }

    tim3_enable_counter(false);

}

void adc_init()
{
    rcc_periph_clock_enable(RCC_DMA);
    nvic_set_priority(NVIC_DMA1_CHANNEL1_IRQ, 3);
    nvic_enable_irq(NVIC_DMA1_CHANNEL1_IRQ);

    adc_setup();

    dma_setup(adc_values, ADC_CHAN_CNT*ADC_FILETER_SIZE);

    adc_enable_overrun_interrupt(ADC1);

    adc_enable_dma(ADC1);

    ADC_CFGR1(ADC1) = (ADC_CFGR1(ADC1) & ~(0x3<<10)) | (0x1<<10);	// Hardware trigger detection on the rising edge
    ADC_CFGR1(ADC1) = (ADC_CFGR1(ADC1) & ~ADC_CFGR1_EXTSEL) | (3<<ADC_CFGR1_EXTSEL_SHIFT);	// toggle by tim3

    tim3_setup();

    adc_start_conversion_regular(ADC1);


    tim3_enable_counter(true);

    delay_ms(100);

}

void tim3_isr(void)
{
    if (timer_get_flag(TIM3, TIM_SR_UIF)) {

        /* Clear compare interrupt flag. */
        timer_clear_flag(TIM3, TIM_SR_UIF);
    }
}

void adc_sample(void)
{
    uint32_t sum_val1 = 0;
    uint32_t sum_val2 = 0;
    uint32_t sum_val3 = 0;
    uint32_t sum_val4 = 0;

    for(int i=0; i<ADC_FILETER_SIZE; i++){
        sum_val1 += adc_values[ADC_CHAN_CNT*i + 0];
        sum_val2 += adc_values[ADC_CHAN_CNT*i + 1];
        sum_val3 += adc_values[ADC_CHAN_CNT*i + 2];
        sum_val4 += adc_values[ADC_CHAN_CNT*i + 3];
    }

    uint32_t filter_val1 = sum_val1/ADC_FILETER_SIZE;
    uint32_t filter_val2 = sum_val2/ADC_FILETER_SIZE;
    uint32_t filter_val3 = sum_val3/ADC_FILETER_SIZE;
    uint32_t filter_val4 = sum_val4/ADC_FILETER_SIZE;

    printf("adc:%d  %d  %d  %d\r\n", filter_val1, filter_val2, filter_val3, filter_val4);

    tim3_enable_counter(true);
}

int main(void)
{
    sys_clock_setup();

    uart_setup();

    printf("adc init.\r\n");

    adc_init(); 

    while(1){
        adc_sample();

        delay_ms(500);
    }

    while(1){/*do nothing*/}///!!!not go here.
    return 0;
}