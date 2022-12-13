/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * Author: Sam Protsenko <joe.skb7@gmail.com>
 *         Mark Sungurov <mark.sungurov@gmail.com>
 */

#ifndef DRIVERS_DS18B20_H
#define DRIVERS_DS18B20_H


#include <stdint.h>

#include "one_wire.h"

/* Contains parsed data from DS18B20 temperature sensor */
struct ds18b20_temp {
	uint16_t integer:12;
	uint16_t frac;
	uint16_t value;
	char sign;		/* '-' or '+' */
};

struct ds18b20 {
	uint32_t port;
	uint16_t pin;
	struct ds18b20_temp temp;
};

int ds18b20_init(struct ds18b20 *obj);
void ds18b20_exit(struct ds18b20 *obj);
struct ds18b20_temp ds18b20_read_temp(struct ds18b20 *obj);
char *ds18b20_temp2str(struct ds18b20_temp *obj, char str[]);

#endif /* DRIVERS_DS18B20_H */
