/*
 * Ubertooth1 interface, for the Ubertooth One hardware
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This code is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __UBERTOOTH_HW_U1_H__
#define __UBERTOOTH_HW_U1_H__

#include <stdint.h>
#include "spectool_container.h"

/* Ubertooth One scan results */
typedef struct _ubertooth_u1_usb_pair {
	uint8_t bus;
	uint8_t dev_addr;
} ubertooth_u1_usb_pair;

int ubertooth_u1_device_scan(spectool_device_list *list);

int ubertooth_u1_init_path(spectool_phy *phydev, uint8_t bus, uint8_t dev_addr);
int ubertooth_u1_init(spectool_phy *phydev, spectool_device_rec *rec);

#endif
