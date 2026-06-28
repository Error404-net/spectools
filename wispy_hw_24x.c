/*
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
 * Extra thanks to Ryan Woodings @ Metageek for interface documentation
 */

#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <limits.h>
#include <math.h>

/* LibUSB 1.0 */
#include <libusb.h>

/* USB HID functions from specs which aren't defined for us */
#define HID_GET_REPORT 0x01
#define HID_SET_REPORT 0x09
#define TIMEOUT	9000

#define METAGEEK_WISPY24x_VID		0x1781
#define METAGEEK_WISPY24x_PID		0x083f

/* Default # of samples */
#define WISPY24x_USB_NUM_SAMPLES		256

/* Set data feature defaults */
#define WISPY24x_USB_DEF_CARRIER		0x58e2f8
#define WISPY24x_USB_DEF_STEP_MANT		0x8e
#define WISPY24x_USB_DEF_STEP_EXP		0x03
#define WISPY24x_USB_DEF_BW				0x05
#define WISPY24x_USB_DEF_BW_AVG			0x01
#define WISPY24x_USB_DEF_STEPS			255
#define WISPY24x_USB_DEF_READS			6
#define WISPY24x_USB_DEF_H_MINKHZ		2399938
#define WISPY24x_USB_DEF_H_RESHZ		327942

#define WISPY24x_USB_CARRIER_C_FROM_KHZ(K)	(int) ceil((float) (K) / \
											((27.0f / pow(2, 16)) * 1000))
#define WISPY24x_USB_CARRIER_KHZ_FROM_C(C)	(int) ((27.0f / pow(2, 16)) * 1000 * (C))
/* Calculate the spacing mantissa from khz */
#define WISPY24x_USB_RES_M_FROM_HZ(H,E)	(int) (((H) / ((27.0f / pow(2, 18)) * \
								 pow(2, (E)) * 1000000)) - 256.0)
#define WISPY24x_USB_RES_HZ_FROM_ME(M,E)	(int) ((27.0f / pow(2, 18)) * (256 + (M)) * \
								 pow(2, (E)) * 1000000)
/* Calculate the bandwidth - Typically we want 3:1 bandwidth:sampleres */
#define WISPY24x_USB_BW_HZ_FROM_ME(M,E)	(int) (27.0f / (8 * (4 + (M)) * \
								 pow(2, (E))) * 1000000)
#define WISPY24x_USB_BW_M_FROM_HZ(H,E)		(int) ((27.0f * 1000000 / ((H) * \
								 8 * pow(2, (E)))) - 4)

#define WISPY24x_USB_BW_EXTRACT_M(D) 	(int) (((D) & 0xC) >> 2)
#define WISPY24x_USB_BW_EXTRACT_E(D)	(int) (((D) & 0x3))
#define WISPY24x_USB_BW_BUILD(M,E)		(int) ((((M) << 2) | ((E))) & 0xF)

#define WISPY24x_USB_OFFSET_MDBM		-134000
#define WISPY24x_USB_RES_MDBM			500
#define WISPY24x_USB_RSSI_MAX			222

/*
#define WISPY24x_USB_MIN_SAMPLE			-125
#define WISPY24x_USB_MIN_SIGNAL			-119
#define WISPY24x_USB_MAX_SAMPLE			-23

#define WISPY24x_USB_RSSI(x)			(((float) (x) * 0.5) - 134.0)
*/

#include "spectool_container.h"
#include "wispy_hw_24x.h"

#define endian_swap32(x) \
({ \
    uint32_t __x = (x); \
    ((uint32_t)( \
        (uint32_t)(((uint32_t)(__x) & (uint32_t)0x000000ff) << 24) | \
        (uint32_t)(((uint32_t)(__x) & (uint32_t)0x0000ff00) << 8) | \
        (uint32_t)(((uint32_t)(__x) & (uint32_t)0x00ff0000) >> 8) | \
        (uint32_t)(((uint32_t)(__x) & (uint32_t)0xff000000) >> 24) )); \
})

static libusb_context *g_usb_ctx = NULL;

/* Aux tracking struct for wispy24x characteristics */
typedef struct _wispy24x_usb_aux {
	libusb_device *dev;
	libusb_device_handle *devhdl;

	time_t last_read;

	/* have we pushed a configure event from sweeps */
	int configured;

	/* IPC tracking records to the forked process for capturing data */
	pthread_t usb_thread;
	int usb_thread_alive;

	/* Has the sweep data buffer been initialized?  (ie, did we get a sample at 0) */
	int sweepbuf_initialized;
	/* how many sweeps has this device done over the run time?  Nice to know, and
	 * we can use it for calibration counters too */
	int num_sweeps;

	/* Sweep buffer we maintain and return */
	spectool_sample_sweep *sweepbuf;

	int sockpair[2];

	int sweepbase;

	spectool_phy *phydev;
} wispy24x_usb_aux;

typedef struct _wispy24x_rfsettings {
	uint8_t feature_id;
	uint32_t freq_carrier : 24;
	uint8_t step_mantissa;
	uint8_t step_exponent;
	uint8_t bandwidth;
	uint8_t bw_average;
	uint8_t num_steps;
	uint8_t num_reads;
	uint32_t freq_min_khz;
	uint32_t freq_res_hz : 24;
} __attribute__((packed)) wispy24x_rfsettings;

typedef struct _wispy24x_report {
	uint8_t report_id;
	uint32_t freq_start_khz;
	uint32_t freq_res_hz : 24;
	uint8_t valid_bytes;
	uint8_t data[54];
} __attribute__((packed)) wispy24x_report;

/* Prototypes */
int wispy24x_usb_open(spectool_phy *);
int wispy24x_usb_close(spectool_phy *);
int wispy24x_usb_thread_close(spectool_phy *);
int wispy24x_usb_poll(spectool_phy *);
int wispy24x_usb_getpollfd(spectool_phy *);
void wispy24x_usb_setcalibration(spectool_phy *, int);
int wispy24x_usb_setposition(spectool_phy *, int, int, int);
spectool_sample_sweep *wispy24x_usb_getsweep(spectool_phy *);

uint32_t wispy24x_adler_checksum(const char *buf1, int len) {
	int i;
	uint32_t s1, s2;
	char *buf = (char *)buf1;
	int CHAR_OFFSET = 0;

	s1 = s2 = 0;
	for (i = 0; i < (len-4); i+=4) {
		s2 += 4*(s1 + buf[i]) + 3*buf[i+1] + 2*buf[i+2] + buf[i+3] + 
			10*CHAR_OFFSET;
		s1 += (buf[i+0] + buf[i+1] + buf[i+2] + buf[i+3] + 4*CHAR_OFFSET); 
	}

	for (; i < len; i++) {
		s1 += (buf[i]+CHAR_OFFSET); s2 += s1;
	}

	return (s1 & 0xffff) + (s2 << 16);
}

/* Scan for devices */
int wispy24x_usb_device_scan(spectool_device_list *list) {
	libusb_device **devlist;
	struct libusb_device_descriptor desc;
	ssize_t cnt, i;
	int num_found = 0;
	wispy24x_usb_pair *auxpair;

	if (!g_usb_ctx)
		libusb_init(&g_usb_ctx);

	cnt = libusb_get_device_list(g_usb_ctx, &devlist);
	if (cnt < 0)
		return 0;

	for (i = 0; i < cnt; i++) {
		if (libusb_get_device_descriptor(devlist[i], &desc) != 0)
			continue;

		if ((desc.idVendor == METAGEEK_WISPY24x_VID) &&
			(desc.idProduct == METAGEEK_WISPY24x_PID)) {

			if (list->num_devs == list->max_devs - 1)
				break;

			auxpair = (wispy24x_usb_pair *) malloc(sizeof(wispy24x_usb_pair));
			auxpair->bus      = libusb_get_bus_number(devlist[i]);
			auxpair->dev_addr = libusb_get_device_address(devlist[i]);

			uint8_t id_buf[2] = { auxpair->bus, auxpair->dev_addr };

			list->list[list->num_devs].device_id =
				wispy24x_adler_checksum((char *)id_buf, 2);
			snprintf(list->list[list->num_devs].name, SPECTOOL_PHY_NAME_MAX,
					 "Wi-Spy 24x USB %u", list->list[list->num_devs].device_id);

			list->list[list->num_devs].init_func = wispy24x_usb_init;
			list->list[list->num_devs].hw_rec = auxpair;

			list->list[list->num_devs].num_sweep_ranges = 1;
			list->list[list->num_devs].supported_ranges =
				(spectool_sample_sweep *) malloc(sizeof(spectool_sample_sweep));

			list->list[list->num_devs].supported_ranges[0].name =
				strdup("2.4GHz ISM");

			list->list[list->num_devs].supported_ranges[0].num_samples =
				WISPY24x_USB_NUM_SAMPLES;

			list->list[list->num_devs].supported_ranges[0].amp_offset_mdbm =
				WISPY24x_USB_OFFSET_MDBM;
			list->list[list->num_devs].supported_ranges[0].amp_res_mdbm =
				WISPY24x_USB_RES_MDBM;
			list->list[list->num_devs].supported_ranges[0].rssi_max =
				WISPY24x_USB_RSSI_MAX;

			list->list[list->num_devs].supported_ranges[0].start_khz =
				WISPY24x_USB_DEF_H_MINKHZ;
			list->list[list->num_devs].supported_ranges[0].end_khz =
				WISPY24x_USB_DEF_H_MINKHZ + ((WISPY24x_USB_DEF_STEPS *
											  WISPY24x_USB_DEF_H_RESHZ) / 1000);
			list->list[list->num_devs].supported_ranges[0].res_hz =
				WISPY24x_USB_DEF_H_RESHZ;

			list->num_devs++;
			num_found++;
		}
	}

	libusb_free_device_list(devlist, 1);
	return num_found;
}

int wispy24x_usb_init(spectool_phy *phydev, spectool_device_rec *rec) {
	wispy24x_usb_pair *auxpair = (wispy24x_usb_pair *) rec->hw_rec;

	if (auxpair == NULL)
		return -1;

	return wispy24x_usb_init_path(phydev, auxpair->bus, auxpair->dev_addr);
}

/* Initialize a specific USB device based on bus number and device address */
int wispy24x_usb_init_path(spectool_phy *phydev, uint8_t bus, uint8_t dev_addr) {
	libusb_device **devlist;
	libusb_device *found = NULL;
	struct libusb_device_descriptor desc;
	ssize_t cnt, i;
	uint8_t id_buf[2] = { bus, dev_addr };
	uint32_t cid;

	wispy24x_usb_aux *auxptr = NULL;

	if (!g_usb_ctx)
		libusb_init(&g_usb_ctx);

	cid = wispy24x_adler_checksum((char *)id_buf, 2);

	cnt = libusb_get_device_list(g_usb_ctx, &devlist);
	if (cnt < 0) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "WISPY24x_INIT failed to enumerate USB devices");
		return -1;
	}

	for (i = 0; i < cnt; i++) {
		if (libusb_get_bus_number(devlist[i]) != bus)
			continue;
		if (libusb_get_device_address(devlist[i]) != dev_addr)
			continue;
		if (libusb_get_device_descriptor(devlist[i], &desc) != 0)
			continue;
		if ((desc.idVendor == METAGEEK_WISPY24x_VID) &&
			(desc.idProduct == METAGEEK_WISPY24x_PID)) {
			found = libusb_ref_device(devlist[i]);
			break;
		} else {
			snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
					 "WISPY24x_INIT failed, specified device %u does not "
					 "appear to be a Wi-Spy device", cid);
			libusb_free_device_list(devlist, 1);
			return -1;
		}
	}

	libusb_free_device_list(devlist, 1);

	if (found == NULL) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "WISPY24x_INIT failed, specified device %u does not appear "
				 "to exist.", cid);
		return -1;
	}

	/* Build the device record with one sweep capability */
	phydev->device_spec = (spectool_dev_spec *) malloc(sizeof(spectool_dev_spec));

	phydev->device_spec->device_id = cid;

	/* Default the name to the buspath */
	snprintf(phydev->device_spec->device_name, SPECTOOL_PHY_NAME_MAX,
			 "Wi-Spy 24x USB %u", cid);

	/* State */
	phydev->state = SPECTOOL_STATE_CLOSED;

	phydev->min_rssi_seen = -1;

	phydev->device_spec->device_version = 0x02;
	phydev->device_spec->device_flags = SPECTOOL_DEV_FL_VAR_SWEEP;

	phydev->device_spec->num_sweep_ranges = 1;
	phydev->device_spec->supported_ranges =
		(spectool_sample_sweep *) malloc(sizeof(spectool_sample_sweep));

	phydev->device_spec->default_range = phydev->device_spec->supported_ranges;

	phydev->device_spec->default_range->name = strdup("2.4GHz Normal");

	phydev->device_spec->default_range->num_samples = WISPY24x_USB_NUM_SAMPLES;

	phydev->device_spec->default_range->amp_offset_mdbm = WISPY24x_USB_OFFSET_MDBM;
	phydev->device_spec->default_range->amp_res_mdbm = WISPY24x_USB_RES_MDBM;
	phydev->device_spec->default_range->rssi_max = WISPY24x_USB_RSSI_MAX;

	phydev->device_spec->default_range->start_khz = WISPY24x_USB_DEF_H_MINKHZ;
	phydev->device_spec->default_range->end_khz = 
		WISPY24x_USB_DEF_H_MINKHZ + ((WISPY24x_USB_DEF_STEPS *
									  WISPY24x_USB_DEF_H_RESHZ) / 1000);
	phydev->device_spec->default_range->res_hz = WISPY24x_USB_DEF_H_RESHZ;

	phydev->device_spec->cur_profile = 0;

	/* Set up the aux state */
	auxptr = malloc(sizeof(wispy24x_usb_aux));
	phydev->auxptr = auxptr;

	auxptr->configured = 0;

	auxptr->dev = found;
	auxptr->devhdl = NULL;
	auxptr->phydev = phydev;
	auxptr->sockpair[0] = -1;
	auxptr->sockpair[1] = -1;

	/* Will be filled in by setposition later */
	auxptr->sweepbuf_initialized = 0;
	auxptr->sweepbuf = NULL;

	phydev->open_func = &wispy24x_usb_open;
	phydev->close_func = &wispy24x_usb_close;
	phydev->poll_func = &wispy24x_usb_poll;
	phydev->pollfd_func = &wispy24x_usb_getpollfd;
	phydev->setcalib_func = &wispy24x_usb_setcalibration;
	phydev->getsweep_func = &wispy24x_usb_getsweep;
	phydev->setposition_func = &wispy24x_usb_setposition;

	phydev->draw_agg_suggestion = 1;

	return 0;
}

void *wispy24x_usb_servicethread(void *aux) {
	wispy24x_usb_aux *auxptr = (wispy24x_usb_aux *) aux;

	int sock;
	libusb_device_handle *wispy;

	char buf[64];
	int x = 0, error = 0;
	fd_set wset;

	struct timeval tm;

	sigset_t signal_set;

	error = 0;

	sock = auxptr->sockpair[1];

	wispy = auxptr->devhdl;

	/* We don't want to see any signals in the child thread */
	sigfillset(&signal_set);
	pthread_sigmask(SIG_BLOCK, &signal_set, NULL);

	while (1) {
		/* wait until we're able to write out to the IPC socket, go into a blocking
		 * select */
		FD_ZERO(&wset);
		FD_SET(sock, &wset);

		if (select(sock + 1, NULL, &wset, NULL, NULL) < 0) {
			snprintf(auxptr->phydev->errstr, SPECTOOL_ERROR_MAX,
					 "wispy24x_usb poller failed on IPC write select(): %s",
					 strerror(errno));
			auxptr->usb_thread_alive = 0;
			auxptr->phydev->state = SPECTOOL_STATE_ERROR;
			pthread_exit(NULL);
		}

		if (auxptr->usb_thread_alive == 0) {
			auxptr->phydev->state = SPECTOOL_STATE_ERROR;
			pthread_exit(NULL);
		}

		if (FD_ISSET(sock, &wset) == 0)
			continue;

		/* Get new data only if we haven't requeued */
		if (error == 0) {
			int ret, xferred = 0;
			memset(buf, 0, 64);

			ret = libusb_interrupt_transfer(wispy, 0x81, (uint8_t *)buf, 64,
											&xferred, TIMEOUT);
			if (ret < 0) {
				if (ret == LIBUSB_ERROR_TIMEOUT)
					continue;

				snprintf(auxptr->phydev->errstr, SPECTOOL_ERROR_MAX,
						 "wispy24x_usb poller failed to read USB data: %s",
						 libusb_error_name(ret));
				auxptr->usb_thread_alive = 0;
				auxptr->phydev->state = SPECTOOL_STATE_ERROR;
				pthread_exit(NULL);
			}

			/* Send it to the IPC remote, re-queue on enobufs */
			if (send(sock, buf, 64, 0) < 0) {
				if (errno == ENOBUFS) {
					error = 1;
					continue;
				}

				snprintf(auxptr->phydev->errstr, SPECTOOL_ERROR_MAX,
						 "wispy24x_usb poller failed on IPC send: %s",
						 strerror(errno));
				auxptr->usb_thread_alive = 0;
				auxptr->phydev->state = SPECTOOL_STATE_ERROR;
				pthread_exit(NULL);
			}

		}

		error = 0;
	}

	auxptr->usb_thread_alive = 0;
	send(sock, buf, 64, 0);
	auxptr->phydev->state = SPECTOOL_STATE_ERROR;
	pthread_exit(NULL);
}

int wispy24x_usb_getpollfd(spectool_phy *phydev) {
	wispy24x_usb_aux *auxptr = (wispy24x_usb_aux *) phydev->auxptr;

	if (auxptr->usb_thread_alive == 0) {
		wispy24x_usb_close(phydev);
		return -1;
	}

	return auxptr->sockpair[0];
}

int wispy24x_usb_open(spectool_phy *phydev) {
	int pid_status;
	wispy24x_usb_aux *auxptr = (wispy24x_usb_aux *) phydev->auxptr;

	/* Make the client/server socketpair */
	if (socketpair(PF_UNIX, SOCK_DGRAM, 0, auxptr->sockpair) < 0) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy24x_usb open failed to create socket pair for capture "
				 "process: %s", strerror(errno));
		return -1;
	}

	int ret;
	ret = libusb_open(auxptr->dev, &auxptr->devhdl);
	if (ret != LIBUSB_SUCCESS) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy24x_usb capture process failed to open USB device: %s",
				 libusb_error_name(ret));
		return -1;
	}

	ret = libusb_detach_kernel_driver(auxptr->devhdl, 0);
	if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_NOT_FOUND &&
		ret != LIBUSB_ERROR_NOT_SUPPORTED) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy24x_usb could not detach kernel driver: %s",
				 libusb_error_name(ret));
	}

	libusb_set_configuration(auxptr->devhdl, 1);

	ret = libusb_claim_interface(auxptr->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy24x_usb could not claim interface: %s",
				 libusb_error_name(ret));
		libusb_close(auxptr->devhdl);
		auxptr->devhdl = NULL;
		return -1;
	}

	auxptr->usb_thread_alive = 1;

	auxptr->last_read = time(0);

	if (pthread_create(&(auxptr->usb_thread), NULL, 
					   wispy24x_usb_servicethread, auxptr) < 0) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy24x_usb capture failed to create thread: %s",
				 strerror(errno));
		auxptr->usb_thread_alive = 0;
		return -1;
	}

	/* Update the state */
	phydev->state = SPECTOOL_STATE_CONFIGURING;

	/*
	if (wispy24x_usb_setposition(phydev, 0, 0, 0) < 0)
		return -1;
	*/

	return 1;
}

int wispy24x_usb_close(spectool_phy *phydev) {
	wispy24x_usb_aux *aux;
	
	if (phydev == NULL)
		return 0;

	aux = (wispy24x_usb_aux *) phydev->auxptr;

	if (aux == NULL)
		return 0;

	/* If the thread is still alive, don't take away the devices it might
	 * still be reading, wait for it to error down */
	if (aux->usb_thread_alive) {
		aux->usb_thread_alive = 0;
		pthread_join(aux->usb_thread, NULL);
	}

	if (aux->devhdl) {
		libusb_release_interface(aux->devhdl, 0);
		libusb_close(aux->devhdl);
		aux->devhdl = NULL;
	}

	if (aux->dev) {
		libusb_unref_device(aux->dev);
		aux->dev = NULL;
	}

	if (aux->sockpair[0] >= 0) {
		close(aux->sockpair[0]);
		aux->sockpair[0] = -1;
	}

	if (aux->sockpair[1] >= 0) {
		close(aux->sockpair[1]);
		aux->sockpair[1] = -1;
	}

	return 1;
}

spectool_sample_sweep *wispy24x_usb_getsweep(spectool_phy *phydev) {
	wispy24x_usb_aux *auxptr = (wispy24x_usb_aux *) phydev->auxptr;

	return auxptr->sweepbuf;
}

void wispy24x_usb_setcalibration(spectool_phy *phydev, int in_calib) {
	phydev->state = SPECTOOL_STATE_RUNNING;
}

int wispy24x_usb_poll(spectool_phy *phydev) {
	wispy24x_usb_aux *auxptr = (wispy24x_usb_aux *) phydev->auxptr;
	char lbuf[64];
	int base, res, ret, x;
	wispy24x_report *report;

	/* Push a configure event before anything else */
	if (auxptr->configured == 0) {
		auxptr->configured = 1;
		return SPECTOOL_POLL_CONFIGURED;
	}

	/* Use the error set by the polling thread */
	if (auxptr->usb_thread_alive == 0) {
		phydev->state = SPECTOOL_STATE_ERROR;
		wispy24x_usb_close(phydev);
		return SPECTOOL_POLL_ERROR;
	}

	if ((ret = recv(auxptr->sockpair[0], lbuf, 64, 0)) < 0) {
		if (auxptr->usb_thread_alive != 0)
			snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
					 "wispy24x_usb IPC receiver failed to read signal data: %s",
					 strerror(errno));
		phydev->state = SPECTOOL_STATE_ERROR;
		return SPECTOOL_POLL_ERROR;
	}

	if (time(0) - auxptr->last_read > 3) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy1_usb didn't see any data for more than 3 seconds, "
				 "something has gone wrong (was the device removed?)");
		phydev->state = SPECTOOL_STATE_ERROR;
		return SPECTOOL_POLL_ERROR;
	}

	if (ret > 0)
		auxptr->last_read = time(0);

	// If we don't have a sweepbuf we're not configured, barf
	if (auxptr->sweepbuf == NULL) {
		return SPECTOOL_POLL_NONE;
	}

	if (ret < sizeof(wispy24x_report)) {
		printf("Short report\n");
		return SPECTOOL_POLL_NONE;
	}

	report = (wispy24x_report *) lbuf;

	/* Derive the base slot from the khz start of this sample, based on
	 * our sweep record */
#ifdef WORDS_BIGENDIAN
	base = endian_swap32(report->freq_start_khz) - auxptr->sweepbuf->start_khz;
	res = endian_swap32(report->freq_res_hz) >> 8;
#else
	base = report->freq_start_khz - auxptr->sweepbuf->start_khz;
	res = report->freq_res_hz;
#endif

	/*
	if (base != 0)
		base = (base) / ((float) auxptr->sweepbuf->res_hz / 1000);
	*/
	
	if (base == 0)
		auxptr->sweepbase = 0;
	else
		base = auxptr->sweepbase;

	if (base < 0 || base > auxptr->sweepbuf->num_samples) {
		/* Bunk data, throw it out */
		return SPECTOOL_POLL_NONE;
	}

	/* Initialize the sweep buffer when we get to it 
	 * If we haven't gotten around to a 0 state to initialize the buffer, we throw
	 * out the sample data until we do. */
	if (base == 0) {
		auxptr->sweepbuf_initialized = 1;
		auxptr->num_sweeps++;

		/* Init the timestamp for sweep begin */
		gettimeofday(&(auxptr->sweepbuf->tm_start), NULL);
	} else if (auxptr->sweepbuf_initialized == 0) {
		return SPECTOOL_POLL_NONE;
	}

	for (x = 0; x < report->valid_bytes; x++) {
		if (base + x >= auxptr->sweepbuf->num_samples) {
			break;
		}

		/*
		auxptr->sweepbuf->sample_data[base + x] =
			WISPY24x_USB_RSSI(report->data[x]);
		*/
		auxptr->sweepbuf->sample_data[base + x] = report->data[x];

		if (report->data[x] < phydev->min_rssi_seen)
			phydev->min_rssi_seen = report->data[x];
	}

	auxptr->sweepbase += report->valid_bytes;

	/* Flag that a sweep is complete */
	if (base + report->valid_bytes == auxptr->sweepbuf->num_samples) {
		gettimeofday(&(auxptr->sweepbuf->tm_end), NULL);
		auxptr->sweepbuf->min_rssi_seen = phydev->min_rssi_seen;
		return SPECTOOL_POLL_SWEEPCOMPLETE;
	}

	return SPECTOOL_POLL_NONE;
}

int wispy24x_usb_setposition(spectool_phy *phydev, int in_profile, 
							 int start_khz, int res_hz) {
	int temp_d, temp_m;
	int best_s_m = 0, best_s_e = 0, best_b_m = 0, best_b_e = 0;
	int m = 0, e = 0, best_d;
	int target_bw;
	libusb_device_handle *wispy;
	wispy24x_rfsettings rfset;
	wispy24x_usb_aux *auxptr = (wispy24x_usb_aux *) phydev->auxptr;

	/* This is totally broken for anything but the default */
	if (in_profile == 0) {
		start_khz = WISPY24x_USB_DEF_H_MINKHZ;
		res_hz = WISPY24x_USB_DEF_H_RESHZ;
		best_s_m = WISPY24x_USB_DEF_STEP_MANT;
		best_s_e = WISPY24x_USB_DEF_STEP_EXP;
		best_b_m = WISPY24x_USB_BW_EXTRACT_M(WISPY24x_USB_DEF_BW);
		best_b_e = WISPY24x_USB_BW_EXTRACT_E(WISPY24x_USB_DEF_BW);
	} else {
		/* Brute-force search the exponents and use the reverse of the function as the
		 * fitness test.  Pick the resolution which is closest to our requested one. */
		best_d = INT_MAX;
		for (e = 0; e <= 4; e++) {
			temp_m = WISPY24x_USB_RES_M_FROM_HZ(res_hz, e) & 0xFF;
			temp_d = abs(res_hz - WISPY24x_USB_RES_HZ_FROM_ME(temp_m, e));

			if (temp_d < best_d) {
				best_d = temp_d;
				best_s_m = temp_m;
				best_s_e = e;
			}
		}

		/* Brute force the bandwidth w/ the mantissa and exponent again and look for the 
		 * bandwidth closest to what we want.
		 * TODO - add bw allocation sliding based on requested resolution, suggested is 
		 * 1:1 for higher ranges of the resolution and 3:1 for lower ranges
		 */
		if (res_hz < 100000)
			target_bw = res_hz * 3;
		else if (res_hz < 200000)
			target_bw = res_hz * 2;
		else
			target_bw = res_hz;
		best_d = INT_MAX;
		for (m = 0; m <= 4; m++) {
			for (e = 0; e <= 4; e++) {
				temp_d = abs(target_bw - WISPY24x_USB_BW_HZ_FROM_ME(m, e));

				if (temp_d < best_d) {
					best_d = temp_d;
					best_b_m = m;
					best_b_e = e;
				}
			}
		}

		/* Reset start_khz to the derived/reversed startkhz to account for any rounding
		 * in the function */
		start_khz = 
			WISPY24x_USB_CARRIER_KHZ_FROM_C(WISPY24x_USB_CARRIER_C_FROM_KHZ(start_khz));
		/* reset the res_hz to the derived value */
		res_hz = WISPY24x_USB_RES_HZ_FROM_ME(best_s_m, best_s_e);
	}

	/* Initialize the hw sweep features */
	rfset.feature_id = 0x02;

	/* Multibytes have to be handled in USB-endian (little) */
#ifdef WORDS_BIGENDIAN
	/* 24-bit values get shifted over to align properly */
	rfset.freq_carrier = endian_swap32(WISPY24x_USB_CARRIER_C_FROM_KHZ(start_khz)) >> 8;
	rfset.freq_min_khz = endian_swap32(start_khz);
	rfset.freq_res_hz = endian_swap32(res_hz) >> 8;
#else
	rfset.freq_carrier = WISPY24x_USB_CARRIER_C_FROM_KHZ(start_khz);
	rfset.freq_min_khz = start_khz;
	rfset.freq_res_hz = res_hz;
#endif

	rfset.step_mantissa = best_s_m;
	rfset.step_exponent = best_s_e;
	rfset.bandwidth = WISPY24x_USB_BW_BUILD(best_b_m, best_b_e);
	rfset.bw_average = WISPY24x_USB_DEF_BW_AVG;
	rfset.num_steps = WISPY24x_USB_DEF_STEPS;
	rfset.num_reads = WISPY24x_USB_DEF_READS;

	wispy = auxptr->devhdl;

	{
		int ret = libusb_control_transfer(wispy,
				LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
				HID_SET_REPORT,
				0x02 + (0x03 << 8),
				0,
				(uint8_t *) &rfset, (int) sizeof(wispy24x_rfsettings),
				TIMEOUT);
		if (ret < 0) {
			snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
					 "wispy24x_usb setposition failed to set sweep feature set: %s",
					 libusb_error_name(ret));
			phydev->state = SPECTOOL_STATE_ERROR;
			return -1;
		}
	}

	/* If we successfully configured the hardware, update the sweep capabilities and
	 * the sweep buffer and reset the device */

	phydev->device_spec->num_sweep_ranges = 1;
	if (phydev->device_spec->supported_ranges)
		free(phydev->device_spec->supported_ranges);
	phydev->device_spec->supported_ranges = 
		(spectool_sample_sweep *) malloc(SPECTOOL_SWEEP_SIZE(0));
	memset (phydev->device_spec->supported_ranges, 0, SPECTOOL_SWEEP_SIZE(0));

	/* Set the universal values */
	phydev->device_spec->supported_ranges[0].num_samples = WISPY24x_USB_NUM_SAMPLES;

	phydev->device_spec->supported_ranges[0].amp_offset_mdbm = WISPY24x_USB_OFFSET_MDBM;
	phydev->device_spec->supported_ranges[0].amp_res_mdbm = WISPY24x_USB_RES_MDBM;
	phydev->device_spec->supported_ranges[0].rssi_max = WISPY24x_USB_RSSI_MAX;

	/* Set the sweep records based on default or new data */
	if (start_khz == 0) {
		phydev->device_spec->supported_ranges[0].start_khz = WISPY24x_USB_DEF_H_MINKHZ;
		phydev->device_spec->supported_ranges[0].end_khz = 
			WISPY24x_USB_DEF_H_MINKHZ + ((WISPY24x_USB_NUM_SAMPLES *
										  WISPY24x_USB_DEF_H_RESHZ) / 1000);
		phydev->device_spec->supported_ranges[0].res_hz = WISPY24x_USB_DEF_H_RESHZ;
	} else {
		phydev->device_spec->supported_ranges[0].start_khz = start_khz;
		phydev->device_spec->supported_ranges[0].end_khz = 
			start_khz + ((WISPY24x_USB_NUM_SAMPLES * res_hz) / 1000);
		phydev->device_spec->supported_ranges[0].res_hz = res_hz;
	}

	/* We're not configured, so we need to push a new configure block out next time
	 * we sweep */
	auxptr->configured = 0;

	/* Rebuild the sweep buffer */
	if (auxptr->sweepbuf)
		free(auxptr->sweepbuf);

	auxptr->sweepbuf =
		(spectool_sample_sweep *) malloc(SPECTOOL_SWEEP_SIZE(WISPY24x_USB_NUM_SAMPLES));
	auxptr->sweepbuf->phydev = phydev;
	auxptr->sweepbuf->start_khz = 
		phydev->device_spec->supported_ranges[0].start_khz;
	auxptr->sweepbuf->end_khz = 
		phydev->device_spec->supported_ranges[0].end_khz;
	auxptr->sweepbuf->res_hz = 
		phydev->device_spec->supported_ranges[0].res_hz;
	auxptr->sweepbuf->num_samples = 
		phydev->device_spec->supported_ranges[0].num_samples;

	auxptr->sweepbuf->amp_offset_mdbm =
		phydev->device_spec->supported_ranges[0].amp_offset_mdbm;
	auxptr->sweepbuf->amp_res_mdbm =
		phydev->device_spec->supported_ranges[0].amp_res_mdbm;
	auxptr->sweepbuf->rssi_max =
		phydev->device_spec->supported_ranges[0].rssi_max;

	/*
	auxptr->sweepbuf->min_sample = 
		phydev->device_spec->supported_ranges[0].min_sample;
	auxptr->sweepbuf->min_sig_report = 
		phydev->device_spec->supported_ranges[0].min_sig_report;
	auxptr->sweepbuf->max_sample =
		phydev->device_spec->supported_ranges[0].max_sample;
	*/

	auxptr->sweepbuf_initialized = 0;
	auxptr->num_sweeps = -1;
}

