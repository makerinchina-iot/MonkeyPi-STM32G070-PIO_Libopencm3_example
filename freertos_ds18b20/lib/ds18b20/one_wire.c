// SPDX-License-Identifier: GPL-3.0-or-later
/*
 * Author: Sam Protsenko <joe.skb7@gmail.com>
 *         Mark Sungurov <mark.sungurov@gmail.com>
 */


#include <libopencm3/stm32/gpio.h>
#include <stddef.h>

#include "one_wire.h"

/* 1-wire specific delay timings (all units are usec) */
#define OW_PRESENCE_WAIT_TIME		70
#define OW_READ_INIT_TIME		5
#define OW_READ_PAUSE			50
#define OW_READ_SAMPLING_TIME		5
#define OW_RESET_TIME			500
#define OW_SLOT_WINDOW			5
#define OW_WRITE_0_TIME			60
#define OW_WRITE_1_PAUSE		50
#define OW_WRITE_1_TIME			10

/* Write bit on 1-wire interface. Caller must disable interrupts*/
static void ow_write_bit(struct ow *obj, uint8_t bit)
{
	gpio_clear(obj->port, obj->pin);
	udelay(bit ? OW_WRITE_1_TIME : OW_WRITE_0_TIME);
	gpio_set(obj->port, obj->pin);
	if (bit)
		udelay(OW_WRITE_1_PAUSE);
}

/* Read bit on 1-wire interface. Caller must disable interrupts */
static uint16_t ow_read_bit(struct ow *obj)
{
	uint16_t bit = 0;



	gpio_clear(obj->port, obj->pin);


	udelay(OW_READ_INIT_TIME);
	
	gpio_set(obj->port, obj->pin);

    // gpio_mode_setup(GPIOA,GPIO_MODE_INPUT,GPIO_PUPD_NONE,GPIO0);

	udelay(OW_READ_SAMPLING_TIME);
	
	bit = gpio_get(obj->port, obj->pin);
	udelay(OW_READ_PAUSE);

    // gpio_mode_setup(GPIOA,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,GPIO0);

	return ((bit != 0) ? 1 : 0);
}

/**
 * Initialize one-wire interface.
 *
 * @param obj Structure to store corresponding GPIOs
 * @return 0 on success or negative code on error
 */
int ow_init(struct ow *obj)
{
	/*
	 * Handle possible case when pin is "0" after startup. Otherwise we
	 * might find ourselves in the middle of the reset pulse.
	 */
	gpio_set(obj->port, obj->pin);
	udelay(OW_RESET_TIME);

	/* If device holds bus in low level -- return error */
	if (!(gpio_get(obj->port, obj->pin)))
		return -1;

	return ow_reset_pulse(obj);
}

/* Destroy object */
void ow_exit(struct ow *obj)
{
	gpio_clear(obj->port, obj->pin);
	UNUSED(obj);
}

/**
 * Reset-presence pulse.
 *
 * @param obj Structure to store corresponding GPIOs
 * @return 0 on success or -1 if device doesn't respond
 */
int ow_reset_pulse(struct ow *obj)
{
	unsigned long flags;
	int val;

	enter_critical(flags);
	gpio_clear(obj->port, obj->pin);
	udelay(OW_RESET_TIME);
	gpio_set(obj->port, obj->pin);
	udelay(OW_PRESENCE_WAIT_TIME);
	val = gpio_get(obj->port, obj->pin);
	udelay(OW_RESET_TIME);
	exit_critical(flags);

	if (val)
		return -1;

	return 0;
}

/**
 * Write byte of data.
 *
 * @param obj Structure to store corresponding GPIOs
 * @param byte Data to be written
 */
void ow_write_byte(struct ow *obj, uint8_t byte)
{
	unsigned long flags;
	size_t i;

	enter_critical(flags);
	for (i = 0; i < 8; i++) {
		ow_write_bit(obj, byte >> i & 1);
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
int8_t ow_read_byte(struct ow *obj)
{
	unsigned long flags;
	int16_t byte = 0;
	size_t i;

	enter_critical(flags);
	for (i = 0; i < 8; i++) {
		byte |= ow_read_bit(obj) << i;
		udelay(OW_SLOT_WINDOW);
	}
	exit_critical(flags);

	return (int8_t)byte;
}
