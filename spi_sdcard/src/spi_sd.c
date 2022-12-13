/**
 * @file spi_sd.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-09-18
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "spi_sd.h"

#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

#define SDSPI    SPI1
#define SD_CS    GPIO4
#define SD_PORT  GPIOA

#define spi_cs_deselect()   gpio_set(SD_PORT, SD_CS)
#define spi_cs_select()     gpio_clear(SD_PORT, SD_CS)


/* Definitions for MMC/SDC command */
#define CMD0	(0x40+0)	/* GO_IDLE_STATE */
#define CMD1	(0x40+1)	/* SEND_OP_COND (MMC) */
#define ACMD41	(0xC0+41)	/* SEND_OP_COND (SDC) */
#define CMD8	(0x40+8)	/* SEND_IF_COND */
#define CMD9	(0x40+9)	/* SEND_CSD */
#define CMD10	(0x40+10)	/* SEND_CID */
#define CMD12	(0x40+12)	/* STOP_TRANSMISSION */
#define ACMD13	(0xC0+13)	/* SD_STATUS (SDC) */
#define CMD16	(0x40+16)	/* SET_BLOCKLEN */
#define CMD17	(0x40+17)	/* READ_SINGLE_BLOCK */
#define CMD18	(0x40+18)	/* READ_MULTIPLE_BLOCK */
#define CMD23	(0x40+23)	/* SET_BLOCK_COUNT (MMC) */
#define ACMD23	(0xC0+23)	/* SET_WR_BLK_ERASE_COUNT (SDC) */
#define CMD24	(0x40+24)	/* WRITE_BLOCK */
#define CMD25	(0x40+25)	/* WRITE_MULTIPLE_BLOCK */
#define CMD55	(0x40+55)	/* APP_CMD */
#define CMD58	(0x40+58)	/* READ_OCR */

#define CT_MMC 0x01 /* MMC ver 3 */
#define CT_SD1 0x02 /* SD ver 1 */
#define CT_SD2 0x04 /* SD ver 2 */
#define CT_SDC (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK 0x08 /* Block addressing */

static uint8_t spi_read_write8(uint32_t spi, uint8_t tx);
static uint8_t wait_ready(void);
static uint8_t send_cmd (uint8_t cmd,uint32_t arg);
static void set_spi_slow();
static void set_spi_fast();

/**
 * @brief init sd card
 * 
 */
uint8_t spi_sd_init()
{
    uint8_t n, cmd, ty, ocr[4];
	uint16_t i;


    //init with low speed
    // set_spi_slow();

	spi_cs_select();

	for (n = 10; n; n--) spi_read_write8(SDSPI,0xff);	/* 80 dummy clocks */
	
	ty = 0;
	
	/* Enter Idle state */
	ty = send_cmd(CMD0, 0);

	// printf("  > enter idle:%d\n", ty);

	/* Initialization timeout of 1000 milliseconds */
    /* SDHC */

    if(ty == 1){
     
        if (send_cmd(CMD8, 0x1AA) == 1){ /* SDv2? */
        /* Get trailing return value of R7 response */
            for (n = 0; n < 4; n++) ocr[n] = spi_read_write8(SDSPI,0xff);
            /* The card can work at VDD range of 2.7-3.6V */
            if (ocr[2] == 0x01 && ocr[3] == 0xAA){
                /* Wait for leaving idle state (ACMD41 with HCS bit) */
                i=0xfff;
                while (--i && send_cmd(ACMD41, 1UL << 30));
                if (i && send_cmd(CMD58, 0) == 0){
                /* Check CCS bit in the OCR */
                    for (n = 0; n < 4; n++) ocr[n] = spi_read_write8(SDSPI,0xff);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                    
                }
            }
        }  else {	/* Not SDv2 card */
			// if (send_cmd(ACMD41, 0) <= 1) 	{	/* SDv1 or MMC? */
			// 	ty = CT_SD1; cmd = ACMD41;	/* SDv1 (ACMD41(0)) */
			// } else {
			// 	ty = CT_MMC; cmd = CMD1;	/* MMCv3 (CMD1(0)) */
			// }

			// while (SPI_Timer_Status() && send_cmd(cmd, 0)) ;		/* Wait for end of initialization */
			// if (!SPI_Timer_Status() || send_cmd(CMD16, 512) != 0)	/* Set block length: 512 */
			// 	ty = 0;
		}

    }
	//CardType = ty;
	

	spi_cs_deselect();
	spi_read_write8(SDSPI,0xff);
	while (SPI_SR(SDSPI) & SPI_SR_BSY);

    set_spi_fast();

	return ty;
}

/**
 * @brief spi read sd
 * 
 * @param buff 
 * @param sector 
 */
uint8_t spi_sd_read(uint8_t *buff, uint32_t sector)
{
    uint8_t result;
	uint16_t cnt=0xffff;
	spi_cs_select();	
	result=send_cmd(CMD17, sector); //CMD17 даташит стр 50 и 96
	if (result){spi_cs_deselect(); return 5;} //Выйти, если результат не 0x00
	
	spi_read_write8(SDSPI,0xff);
	cnt=0;
	do result=spi_read_write8(SDSPI,0xff); while ((result!=0xFE)&&--cnt);
	if(!cnt){spi_cs_deselect(); return 5;}
	
	for (cnt=0;cnt<512;cnt++) *buff++=spi_read_write8(SDSPI,0xff); 
	
    spi_read_write8(SDSPI,0xff); 
	spi_read_write8(SDSPI,0xff);
	spi_cs_deselect();
	spi_read_write8(SDSPI,0xff);

	return 0;
}

/**
 * @brief spi write sd
 * 
 * @param buff 
 * @param sector 
 */
uint8_t spi_sd_write(uint8_t *buff, uint32_t sector)
{
    uint8_t result;
	uint16_t cnt=0xffff;
	spi_cs_select();
	result=send_cmd(CMD24,sector); //CMD24 
	if(result){spi_cs_deselect(); return 6;} //
	spi_read_write8(SDSPI,0xff);
	spi_read_write8(SDSPI,0xfe);//
	for (cnt=0;cnt<512;cnt++) spi_read_write8(SDSPI,buff[cnt]); //Данные
	spi_read_write8(SDSPI,0xff);
	spi_read_write8(SDSPI,0xff);
	result=spi_read_write8(SDSPI,0xff);
	//result=wait_ready();
	if((result&0x05)!=0x05){spi_cs_deselect(); return 6;} 
	//spi_read_write8(SDSPI,0xff);
	while (SPI_SR(SDSPI) & SPI_SR_BSY);
	//
	spi_cs_deselect();
	spi_read_write8(SDSPI,0xff);
	return 0;
}


static void set_spi_slow()
{
	// spi_disable(SDSPI);
	spi_set_baudrate_prescaler(SDSPI,SPI_CR1_BAUDRATE_FPCLK_DIV_128);
	// spi_enable(SDSPI);
}

static void set_spi_fast()
{
	// spi_disable(SDSPI);
	spi_set_baudrate_prescaler(SDSPI,SPI_CR1_BAUDRATE_FPCLK_DIV_8);
	// spi_enable(SDSPI);
}

static uint8_t spi_read_write8(uint32_t spi, uint8_t tx)
{
	spi_send8(spi, tx);
	return spi_read8(spi);
}

static uint8_t wait_ready(void)
{
	uint8_t res;
	uint16_t cnt=0xffff;
	spi_read_write8(SDSPI, 0xff);
	do res = spi_read_write8(SDSPI, 0xff); while ((res!=0xFF)&& --cnt );
	return res;
}

static uint8_t send_cmd (uint8_t cmd,uint32_t arg)
{
	uint8_t n, res;

    /* ACMD<n> is the command sequence of CMD55-CMD<n> */
	if (cmd & 0x80){
		cmd &= 0x7F;
		res = send_cmd(CMD55, 0);
		if (res > 1) return res;
	}

	if (wait_ready()!=0xFF) return 0xFF;
	/* Send command packet */
	spi_read_write8(SDSPI, cmd);					/* Start + Command index */
	spi_read_write8(SDSPI,(uint8_t)(arg >> 24));	/* Argument[31..24] */
	spi_read_write8(SDSPI,(uint8_t)(arg >> 16));	/* Argument[23..16] */
	spi_read_write8(SDSPI,(uint8_t)(arg >> 8));	/* Argument[15..8] */
	spi_read_write8(SDSPI,(uint8_t)arg);			/* Argument[7..0] */
	n = 0x01;								/* Dummy CRC + Stop */
	if (cmd == CMD0) n = 0x95;				/* Valid CRC for CMD0(0) */
	if (cmd == CMD8) n = 0x87;				/* Valid CRC for CMD8(0x1AA) */
	spi_read_write8(SDSPI,n);
	/* Receive command response */
	if (cmd == CMD12) spi_read_write8(SDSPI,0xff);	
	/* Skip a stuff byte when stop reading */
    /* Wait for a valid response in timeout of 10 attempts */
	n = 10;
	do res=spi_read_write8(SDSPI,0xff); while ((res & 0x80) && --n);
	
	while (SPI_SR(SDSPI) & SPI_SR_BSY); //wait if busy

	return res;			           /* Return with the response value */
}
