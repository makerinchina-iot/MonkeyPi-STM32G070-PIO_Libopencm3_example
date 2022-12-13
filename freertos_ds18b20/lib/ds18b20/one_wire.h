/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Author: Sam Protsenko <joe.skb7@gmail.com>
 *         Mark Sungurov <mark.sungurov@gmail.com>
 */

#ifndef DRIVERS_ONE_WIRE_H
#define DRIVERS_ONE_WIRE_H

#include <stdint.h>

#include "delay.h"

struct ow {
	uint32_t port;
	uint16_t pin;
};

#define BIT(n)			(1 << (n))
#define ARRAY_SIZE(a)		(sizeof(a) / sizeof(a[0]))
#define UNUSED(x)		((void)x)

/** CPU cycles per 1 iteration of loop in ldelay() */
#define CYCLES_PER_LOOP		3UL
/** How many CPU cycles to wait for 1 usec */
#define CYCLES_PER_USEC		64UL	   /* for 24 MHz CPU frequency */
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



int ow_init(struct ow *obj);
void ow_exit(struct ow *obj);
int ow_reset_pulse(struct ow *obj);
void ow_write_byte(struct ow *obj, uint8_t byte);
int8_t ow_read_byte(struct ow *obj);

#endif /* DRIVERS_ONE_WIRE_H */
