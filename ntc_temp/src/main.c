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
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include <libopencm3/stm32/adc.h>

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

void adc_setup()
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_ADC);

    gpio_mode_setup(GPIOA,GPIO_MODE_ANALOG,GPIO_PUPD_NONE,GPIO0);

    adc_power_off(ADC1);
    adc_set_clk_prescale(ADC1,ADC_CCR_PRESC_DIV2);
    adc_set_single_conversion_mode(ADC1);
    adc_set_right_aligned(ADC1);
    adc_set_sample_time_on_all_channels(ADC1,ADC_SMPTIME_160DOT5);
    uint8_t channel = 0;
    adc_set_regular_sequence(ADC1,1,&channel);
    adc_set_resolution(ADC1,ADC_CFGR1_RES_12_BIT);

    adc_power_on(ADC1);

    delay_ms(10);

}

uint16_t adc_sample()
{
    adc_start_conversion_regular(ADC1);
    return adc_read_regular(ADC1);
}

//from adc to temp
//NTC connect to ground, maximumToMinimum
const uint16_t ntcDATA[] = {
4040,4038,4035,4031,4027,4023,4018,4013,4007,4001,3994,3987,3980,3972,3964,3956,

3947,3937,3928,3917,3907,3896,3884,3872,3860,3847,3833,3819,3804,3789,3773,3757,

3740,3722,3703,3684,3664,3644,3622,3600,3577,3553,3528,3502,3475,3448,3419,3390,

3360,3329,3297,3264,3230,3195,3160,3123,3086,3048,3010,2970,2930,2889,2848,2806,

2763,2720,2676,2633,2588,2544,2499,2454,2408,2363,2317,2272,2226,2181,2136,2091,

2048,2001,1957,1913,1869,1826,1783,1741,1699,1658,1617,1576,1537,1498,1459,1422,

1385,1348,1312,1277,1243,1209,1176,1144,1112,1081,1051,1022,993,965,937,910,

884,859,834,810,786,764,741,720,699,679,659,639,621,602,585,568,

551,535,519,504,490,475,461,448,435,422,410,398,387,376,365,355,

345,335,325,316,307,299,290,282,274,267,259,252,245,239,232,226,

220,214,208,202,197,192,187,182,177,172,168,163,159,155,151,146,

143,139,135,132,129
};

#define TEMP_HEADER_VALUE   -55 //the first temp in table

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])

/**
 * @brief search the table, return a midium value if not found
 * 
 * @param table  the data to search
 * @param len    the table length
 * @param up    if data is min to max
 * @return int32_t -1 -> if not found
 */
int32_t bsearch_ret_mid(const uint16_t *table, uint16_t len, bool up, uint16_t key)
{
    uint16_t bot;
    uint16_t mid;
    uint16_t check;
    uint16_t top;

    if (len == 0) {
        return -1;
    }

    if (up) {
        //the first data will be min
        if (key < table[0]) {
            return -1;
        }

        //bigger then the last data
        if (key > table[len - 1]) {
            return -1;
        }
    } else {
        if (key > table[0]) {
            return -1;
        }

        if (key < table[len - 1]) {
            return -1;
        }
    }

    bot = 0;
    top = len - 1;

    if (up) {
        while (bot < top) {
            mid = top - (top - bot) / 2;

            if (key < table[mid]) {
                top = mid - 1;
            } else {
                bot = mid;
            }
        }
    } else {
        while (bot < top) {
            mid = top - (top - bot) / 2;

            if (key > table[mid]) {
                top = mid - 1;
            } else {
                bot = mid;
            }
        }
    }

    if (key == table[top]) {
        return top;
    }

    //not equal the data in table
    if (up) {
        if (key > table[top]) {
            return top;
        }
    } else {
        if (key < table[top]) {
            return top;
        }
    }

    return -1;
}

//get temperature , x10
int16_t ntc2tem(uint16_t adc)
{
    int32_t  index = 0;
    int16_t temperature = 0;

    index = bsearch_ret_mid(ntcDATA, ITEM_NUM(ntcDATA),false,adc);

    //max, the first value
    if(index==0){
        temperature = TEMP_HEADER_VALUE*10;
    }
    //min, the last value
    else if(index>= ITEM_NUM(ntcDATA)){
        temperature = 10*(TEMP_HEADER_VALUE + ITEM_NUM(ntcDATA));
    }
    else{
        //just get integer number
        // temperature = TEMP_HEADER_VALUE + index;

        //get approximation data
        temperature = TEMP_HEADER_VALUE + index;

        //at middle
        temperature = (temperature+1)*10 - 10.0 * (adc-ntcDATA[index+1])/(ntcDATA[index]-ntcDATA[index+1]);
    }

    return temperature;

}

int main(void)
{
    sys_clock_setup();

    uart_setup();

    printf("init adc.\r\n");
    adc_setup();

    while(1){
        uint16_t adc = adc_sample();

        int16_t temp_int = ntc2tem(adc);
        float temp = temp_int/10.0;

        printf("adc: %d, temp: %.1f\r\n",adc, temp);

        delay_ms(500);
    }

    while(1){/*do nothing*/}///!!!not go here.
    return 0;
}