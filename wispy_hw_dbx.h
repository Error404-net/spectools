/*
 * Wi-Spy DBx interface, for third-gen Wi-Spy hardware
 *
 * 255 samples, variable resolution
 *
 * Device configured via HID feature SET
 * Device read via blocking USB endpoint iread
 *
 * Wi-Spy (tm) metageek LLC
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
 * Extra thanks to all @ Metageek for interface documentation
 */

#ifndef __WISPY_HW_DBX_H__
#define __WISPY_HW_DBX_H__

#include <stdint.h>
#include "spectool_container.h"

/* WiSpy DBx device scan results */
typedef struct _wispydbx_usb_pair {
	uint8_t bus;
	uint8_t dev_addr;
} wispydbx_usb_pair;

int wispydbx_usb_device_scan(spectool_device_list *list);

/* WiSpy DBx init function to build a phydev linked to a bus and device address */
int wispydbx_usb_init_path(spectool_phy *phydev, uint8_t bus, uint8_t dev_addr);
int wispydbx_usb_init(spectool_phy *phydev, spectool_device_rec *rec);

#endif
