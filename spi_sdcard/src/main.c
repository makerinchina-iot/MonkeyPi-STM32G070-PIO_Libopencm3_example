/**
 * @file main.c
 * @author MakerInChina (makerinchina.cn)
 * @brief spi sdcard tutorial
 * @version 0.01
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>

#include "spi_sd.h"
#include "ff.h"

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


static void spi1_init(void){
	//spi1 - display
	/* Enable SPI1 Periph and gpio clocks */
	rcc_periph_clock_enable(RCC_SPI1);
	rcc_periph_clock_enable(RCC_GPIOA);
    
	/* Configure GPIOs:
	 * 
	 * SCK=PA5
	 * MOSI=PA7 
	 * MISO=PA6
	 * 
	 * for SD card
	 * SDCS PA4
	 */
	 
	//MOSI & SCK & MISO
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,GPIO5|GPIO7|GPIO6);
    gpio_set_af(GPIOA,GPIO_AF0,GPIO5|GPIO7|GPIO6);
    gpio_set_output_options(GPIOA, GPIO_OTYPE_PP,GPIO_OSPEED_LOW,GPIO5|GPIO7|GPIO6);

    //SDCS 
    gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO4);
	
	gpio_set(GPIOA,GPIO4);

  /* Reset SPI, SPI_CR1 register cleared, SPI is disabled */
	spi_reset(SPI1);
  
  /* Set up SPI in Master mode with:
   * Clock baud rate
   * Clock polarity
   * Clock phase
   * Frame format MSB
   */
	spi_init_master(SPI1, SPI_CR1_BAUDRATE_FPCLK_DIV_128, 
					SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE,
					SPI_CR1_CPHA_CLK_TRANSITION_1,
					SPI_CR1_MSBFIRST);

    spi_set_data_size(SPI1,SPI_CR2_DS_8BIT);
	spi_set_full_duplex_mode(SPI1);
    
    spi_fifo_reception_threshold_8bit(SPI1);

    // spi_set_dff_8bit(SPI1);//not

    // SPI_CR2(SPI1) |= SPI_CR2_NSSP; //NSSP, ?? clock continus
    // spi_set_unidirectional_mode(SPI1);
    // SPI_CR2(SPI1) &= (~SPI_CR2_FRF_TI_MODE); //motorala mode
    // SPI_CR2(SPI1) |= SPI_CR2_FRF_TI_MODE;

  /*
   * Set NSS management to software.
   *
   * Note:
   * Setting nss high is very important, even if we are controlling 
   * the GPIO
   * ourselves this bit needs to be at least set to 1, otherwise the spi
   * peripheral will not send any data out.
   */
	spi_enable_software_slave_management(SPI1);
	spi_set_nss_high(SPI1);

  /* Enable SPI1 periph. */
	spi_enable(SPI1);

}

int main(void)
{
    sys_clock_setup();

    uart_setup();

    printf(" init spi.\n");

    //spi
    spi1_init();

    //sd
    uint8_t sd_type = spi_sd_init();

    // // sdv2: 0x08|0x04 . 0x0c
    // printf(" sd_type: %x\n", sd_type);

    FATFS fs;
    FRESULT res;
    res = f_mount(&fs, "", 0);
	if(res != FR_OK) {
         printf("mount fs failed, res = %d\r\n", res);
        return -1;
	}else{
        printf(" mount fs OK.\n");
    }

    FIL fd;

    res = f_open(&fd, "test.txt", FA_CREATE_ALWAYS|FA_WRITE);
    if(res != FR_OK){
        printf("open file failed: %d\n", res);
    }else{
        printf("open file OK.\n");
    }

    char *buff = "test data to write to fs\n";
    uint32_t len = 0;
    res = f_write(&fd,buff, strlen(buff),&len);
    if(res != FR_OK){
        printf(" write file failed.\n");
    }else{
        printf(" write file OK, write size %d .\n", len);
    }

    res = f_close(&fd);
    if(res != FR_OK){
        printf(" close fs failed.\n");
    }else{
        printf(" close fs OK.\n");
    }

    res = f_unmount("");
    if(res != FR_OK) {
        printf("Unmount fs failed, res = %d\r\n", res);
        return -1;
	}else{
        printf("Unmound fs OK.\n");
    }

    while(1){}

    return 0;
}


/**
 * @brief uart1 isr function
 * 
 */
void usart1_isr(void)
{
    //receive interrupt
	if (((USART_CR1(USART1) & USART_CR1_RXNEIE) != 0) &&
	    ((USART_ISR(USART1) & USART_ISR_RXNE) != 0)) {

        if(recv_index < BUFF_SIZE){
            recv_buff[recv_index++] = usart_recv(USART1);
        }else{
            recv_index = 0;
        }
	}
}