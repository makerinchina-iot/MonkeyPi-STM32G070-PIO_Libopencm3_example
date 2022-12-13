// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Author: Sam Protsenko <joe.skb7@gmail.com>
 *         Mark Sungurov <mark.sungurov@gmail.com>
 */


#include <libopencm3/stm32/gpio.h>
#include <stddef.h>
#include <string.h>

#include "delay.h"

#include "ds18b20.h"

#define TEMPERATURE_CONV_TIME		900

#define CMD_SKIP_ROM			0xcc
#define CMD_CONVERT_T			0x44
#define CMD_READ_SCRATCHPAD		0xbe

// static struct ow ow;

extern void log(const char* fmt, ...);

/**
 *  Reverse the given null-terminated string in place.
 *
 *  Swap the values in the two given variables
 *  Fails when a and b refer to same memory location
 *  This works because of three basic properties of xor:
 *  x ^ 0 = x, x ^ x = 0 and x ^ y = y ^ x for all values x and y
 *
 *  @param input should be an array, whose contents are initialized to
 *  the given string constant.
 */
static void inplace_reverse(char *str)
{
	if (str) {
		char *end = str + strlen(str) - 1;

#define XOR_SWAP(a,b) do\
		{\
			a ^= b;\
			b ^= a;\
			a ^= b;\
		} while (0)

		/* Walk inwards from both ends of the string,
		 * swapping until we get to the middle
		 */
		while (str < end) {
			XOR_SWAP(*str, *end);
			str++;
			end--;
		}
#undef XOR_SWAP
	}
}

/**
 * Parse temperature register from DS18B20.
 *
 * @param lsb Least significant byte of temperature register
 * @param msb Most significant byte of temperature register
 * @return Parsed value
 */
static struct ds18b20_temp ds18b20_parse_temp(uint8_t lsb, uint8_t msb)
{
	struct ds18b20_temp tv;

	tv.integer = (msb << 4) | (lsb >> 4);
	if (msb & BIT(7)) {
		tv.sign = '-';
		/*
		 * Handle negative 2's complement frac value:
		 *   1. Take first 4 bits from LSB (negative part)
		 *   2. Append missing 1111 bits (0xf0), to be able to invert
		 *      8-bit variable
		 *   3. -1 and invert (to handle 2's complement format),
		 *      accounting for implicit integer promotion rule (by
		 *      casting result to uint8_t)
		 */
		tv.frac = 625 * (uint8_t)(~(((lsb & 0xf) - 1) | 0xf0));
		/* Handle negative 2's complement integer value */
		tv.integer = ~tv.integer;
		if (tv.frac == 0)
			tv.integer++;
	} else {
		tv.sign = '+';
		tv.frac = 625 * (lsb & 0xf);
	}

		//将高低两个字节合成一个整形变量
   uint16_t value = msb;
    value <<= 8;
    value |= lsb;
    //DS18B20的精确度为0.0625度, 即读回数据的最低位代表0.0625度
    float t = value * 0.0625;
    //将它放大10倍, 使显示时可显示小数点后一位, 并对小数点后第二2行4舍5入
    //如t=11.0625, 进行计数后, 得到value = 111, 即11.1 度
    value = t * 10 + 0.5;

	// log("temp=%d\r\n", value);

	tv.value = value;

	return tv;
}

/**
 * Read temperature register from DS18B20.
 *
 * @param obj 1-wire device object
 * @return Parsed value
 */
struct ds18b20_temp ds18b20_read_temp(struct ds18b20 *obj)
{
	unsigned long flags;
	int8_t data[2];
	size_t i;

	struct ow ow;
	ow.pin = obj->pin;
	ow.port = obj->port;

	// log("\r\n >>> ow:%d\r\n", ow.pin);

	ow_reset_pulse(&ow);
	ow_write_byte(&ow, CMD_SKIP_ROM);
	ow_write_byte(&ow, CMD_CONVERT_T);
	enter_critical(flags);
	mdelay(TEMPERATURE_CONV_TIME);
	exit_critical(flags);
	ow_reset_pulse(&ow);
	ow_write_byte(&ow, CMD_SKIP_ROM);
	ow_write_byte(&ow, CMD_READ_SCRATCHPAD);

	for (i = 0; i < 2; i++)
		data[i] = ow_read_byte(&ow);
	ow_reset_pulse(&ow);

	// log("\r\ntemp data:0x %x, %x\r\n", data[0], data[1]);

	obj->temp = ds18b20_parse_temp(data[0], data[1]);
	return obj->temp;
}

/**
 * Convert temperature data into null-terminated string.
 *
 * @param obj Contains parsed temperature register from DS18B20
 * @param str Array to store string literal
 * @return Pointer to composed string
 */
char *ds18b20_temp2str(struct ds18b20_temp *obj, char str[])
{
	int i = 0;
	uint16_t rem;

	if (!obj->frac) {
		str[i++] = '0';
	} else {
		while (obj->frac) {
			rem = obj->frac % 10;
			str[i++] = rem + '0';
			obj->frac /= 10;
		}
	}
	str[i++] = '.';
	if (!obj->integer) {
		str[i++] = '0';
	} else {
		while (obj->integer) {
			rem = obj->integer % 10;
			str[i++] = rem + '0';
			obj->integer /= 10;
		}
	}
	str[i++] = obj->sign;
	str[i] = '\0';
	inplace_reverse(str);

	return str;
}

int ds18b20_init(struct ds18b20 *obj)
{
	struct ow ow;

	ow.port = obj->port;
	ow.pin = obj->pin;

	return ow_init(&ow);
}

/* Destroy ds18b20 object */
void ds18b20_exit(struct ds18b20 *obj)
{
	UNUSED(obj);

	struct ow ow;
	ow.pin = obj->pin;
	ow.port = obj->port;

	ow_exit(&ow);
}
