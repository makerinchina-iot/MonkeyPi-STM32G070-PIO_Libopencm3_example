/**
 * @file ow_ds18b20.c
 * @author MakerInChina (makerinchina.cn)
 * @brief 
 * @version 0.01
 * @date 2022-12-28
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "ow_ds18b20.h"

#include <libopencm3/stm32/gpio.h>

#include <string.h>

/* 1-wire specific delay timings (all units are usec) */
#define OW_PRESENCE_WAIT_TIME		70
#define OW_READ_INIT_TIME		    5
#define OW_READ_PAUSE			    50
#define OW_READ_SAMPLING_TIME		5
#define OW_RESET_TIME			    500
#define OW_SLOT_WINDOW			    5
#define OW_WRITE_0_TIME			    60
#define OW_WRITE_1_PAUSE		    50
#define OW_WRITE_1_TIME			    10

#define TEMPERATURE_CONV_TIME		900

#define CMD_SKIP_ROM			0xcc
#define CMD_CONVERT_T			0x44
#define CMD_READ_SCRATCHPAD		0xbe

/** Delay for "d" micro-seconds */
#define udelay(d)		delay_us(d)
/** Delay for "d" milliseconds */
#define mdelay(d)		udelay((d) * 1000UL)

/**
 * Enter critical section (store IRQ flags and disable interrupts).
 *
 * This function serves a purpose of guarding some code from being interrupted
 * by ISR. For example it can be used to safely read/write variable which is
 * also being changed in ISR. Note that it's not necessary to mark such
 * variables with "volatile" modifier when already guarding it with critical
 * section (refer to @ref exit_critical() documentation for details). Keep the
 * critical section as small and fast as possible!
 *
 * The @p flags parameter indicates whether interrupts are enabled or disabled
 * right now (before disabling interrupts). It's often desired to restore such
 * interrupts enabled/disabled state instead of just enabling interrupts. This
 * function stores IRQ flags into provided variable instead of using global
 * variable, to avoid re-entrance issues.
 *
 * Memory barriers are not needed when disabling interrupts (see AN321).
 *
 * @param[out] flags Will contain IRQ flags value before disabling interrupts;
 *                   must have "unsigned long" type
 */
#define enter_critical(flags)						\
do {									\
	__asm__ __volatile__ (						\
		"mrs %0, primask\n"	/* save PRIMASK to "flags" */	\
		"cpsid i"		/* disable interrupts */	\
		: "=r" (flags)						\
		:							\
		: "memory");						\
} while (0)

/**
 * Exit critical section (restore saved IRQ flags).
 *
 * Restores interrupts state (enabled/disabled) stored in @ref enter_critical().
 *
 * Contains memory barriers:
 *   - compiler barrier ("memory"): to prevent caching the values in registers;
 *     this way we don't have to use "volatile" modifier for variables that
 *     are being changed in ISRs
 *   - processor barrier (ISB): so that pending interrupts are run right after
 *     ISB instruction (as recommended by ARM AN321 guide)
 *
 * @param[in] flags Previously saved IRQ flags.
 */
#define exit_critical(flags)						\
do {									\
	__asm__ __volatile__ (						\
		"msr primask, %0\n" /* load PRIMASK from "flags" */	\
		"isb"							\
		:							\
		: "r" (flags)						\
		: "memory");						\
} while (0)


extern void log(const char* fmt, ...);

/* Write bit on 1-wire interface. Caller must disable interrupts*/
static void ow_write_bit(ow_t obj[], uint8_t num, uint8_t bit)
{
	for(uint8_t i=0;i<num;i++)
	    gpio_clear(obj[i].port, obj[i].pin);
	udelay(bit ? OW_WRITE_1_TIME : OW_WRITE_0_TIME);
	for(uint8_t i=0;i<num;i++)
	    gpio_set(obj[i].port, obj[i].pin);
	if (bit)
		udelay(OW_WRITE_1_PAUSE);
}

/* Read bit on 1-wire interface. Caller must disable interrupts */
static uint16_t ow_read_bit(ow_t obj[], uint8_t num, uint16_t bits[])
{
	uint16_t bit = 0;



	for(uint8_t i=0;i<num;i++)
	    gpio_clear(obj[i].port, obj[i].pin);


	udelay(OW_READ_INIT_TIME);
	
	for(uint8_t i=0;i<num;i++)
	    gpio_set(obj[i].port, obj[i].pin);

	udelay(OW_READ_SAMPLING_TIME);
	
    for(uint8_t i=0;i<num;i++){
	    bit = gpio_get(obj[i].port, obj[i].pin);

        bits[i] = (bit != 0) ? 1 : 0;
    }
	udelay(OW_READ_PAUSE);

	return 0;
}

/**
 * Write byte of data.
 *
 * @param obj Structure to store corresponding GPIOs
 * @param byte Data to be written
 */
void ow_write_byte(ow_t obj[], uint8_t num, uint8_t byte)
{
	unsigned long flags;
	uint8_t i;

	enter_critical(flags);
	for (i = 0; i < 8; i++) {
		ow_write_bit(obj,num, byte >> i & 1);
		udelay(OW_SLOT_WINDOW);
	}
	exit_critical(flags);
}

/**
 * Read byte of data.
 *
 * @param obj Structure to store corresponding GPIOs
 * @return Byte read from scratchpad
 */
int8_t ow_read_byte(ow_t obj[], uint8_t num, int8_t bytes[])
{
	unsigned long flags;
	int16_t byte = 0;
	uint8_t i;

	enter_critical(flags);

    uint16_t bits[32] = {0};
	for (i = 0; i < 8; i++) {

        
        ow_read_bit(obj, num, bits);

		// log("(%d %d %d %d)\r\n", bits[0], bits[1], bits[2], bits[3]);

        for(int x=0; x<num; x++){
            bytes[x] |= bits[x] << i;
        }

        // log("\r\n [%x] [%x] [%x] [%x]\r\n", bytes[0],bytes[1],bytes[2],bytes[3]);

		// byte |= ow_read_bit(obj,num) << i;

		udelay(OW_SLOT_WINDOW);
	}
	exit_critical(flags);

	return 0;
}

int ow_ds18b20_reset(ow_t obj[], uint8_t num)
{
    unsigned long flags;

	enter_critical(flags);
    for(uint8_t i=0;i<num;i++)
	    gpio_clear(obj[i].port, obj[i].pin);
	udelay(OW_RESET_TIME);
    for(uint8_t i=0;i<num;i++)
	    gpio_set(obj[i].port, obj[i].pin);
	udelay(OW_PRESENCE_WAIT_TIME);
    for(uint8_t i=0;i<num;i++)
	    gpio_get(obj[i].port, obj[i].pin);
	udelay(OW_RESET_TIME);
	exit_critical(flags);
}

int ow_ds18b20_init(ow_t obj[], uint8_t num)
{
    uint8_t i=0;
    for(i=0;i<num;i++){
        gpio_set(obj[i].port, obj[i].pin);

    }

    udelay(OW_RESET_TIME);

    ow_ds18b20_reset(obj,num);

}

int ow_ds18b20_conv_temp(ow_t obj[], uint8_t num)
{
	ow_ds18b20_reset(obj,num);
	ow_write_byte(obj,num, CMD_SKIP_ROM);
	ow_write_byte(obj,num, CMD_CONVERT_T);
}

int ow_ds18b20_get_temp(ow_t obj[], uint8_t num)
{

    unsigned long flags;
	int8_t data[2];
	int i;

	// ow_ds18b20_reset(obj,num);
	// ow_write_byte(obj,num, CMD_SKIP_ROM);
	// ow_write_byte(obj,num, CMD_CONVERT_T);

	// enter_critical(flags);
	// mdelay(TEMPERATURE_CONV_TIME);
	// exit_critical(flags);

	ow_ds18b20_reset(obj,num);
	ow_write_byte(obj,num, CMD_SKIP_ROM);
	ow_write_byte(obj,num, CMD_READ_SCRATCHPAD);


    int8_t lsb_data[32]={0};
    int8_t msb_data[32]={0};

	memset(lsb_data,0,32);
    ow_read_byte(obj,num,lsb_data);

	memset(msb_data,0,32);
    ow_read_byte(obj,num,msb_data);

	// ow_ds18b20_reset(obj,num);

  //将高低两个字节合成一个整形变量
  for(int i=0; i<num; i++){
    uint16_t value = (uint8_t)msb_data[i];
    value <<= 8;
    value |= (uint8_t)lsb_data[i];

	log("temp:%d\r\n", value);

    //DS18B20的精确度为0.0625度, 即读回数据的最低位代表0.0625度
    float t = value * 0.0625;
    //将它放大10倍, 使显示时可显示小数点后一位, 并对小数点后第二2行4舍5入
    //如t=11.0625, 进行计数后, 得到value = 111, 即11.1 度
    value = t * 10 + 0.5;

	obj[i].temp = value;

	log("temp%d=%d.%d\r\n",i, value/10, value%10);

  }

//   log("\r\n------------\r\n\r\n");
}
