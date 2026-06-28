/* 
 * Wi-Spy Gen1 USB interface
 *
 * Original USB device, polled via HID FEATURE requests.   84 samples, 1MHz
 * resolution.
 *
 * Userspace driver uses libusb
 *
 * Includes HORRIBLE hack for Linux to attempt to disconnect a device when
 * kernel includes in linux/usbdevice_fs.h are mismatched to current kernel
 * support.
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

/* LibUSB 1.0 */
#include <libusb.h>

/* USB HID functions from specs which aren't defined for us */
#define HID_GET_REPORT 0x01
#define HID_RT_FEATURE 0x03
#define TIMEOUT	3000

/* Wispy1a had a bad vid/pid pair in the flash */
#define METAGEEK_WISPY1A_VID		0x04b4
#define METAGEEK_WISPY1A_PID		0x0bad

/* Wispy1b has it's own vid/pid */
#define METAGEEK_WISPY1B_VID		0x1781
#define METAGEEK_WISPY1B_PID		0x083e

/* Various settings */
#define WISPY1_USB_NUM_SAMPLES			83
#define WISPY1_USB_OFFSET_MDBM				-97000
#define WISPY1_USB_RES_MDBM					1500
#define WISPY1_USB_RSSI_MAX					35
#define WISPY1_USB_RSSI_CALIBRATE_THRESH	4
/* calibration in RSSI, one dB approx 2 RSSI */
#define WISPY1_USB_CALIBRATE_MAX			1 
#define WISPY1_USB_CALIBRATE_SWEEPS			10

#include "spectool_container.h"
#include "wispy_hw_gen1.h"

static libusb_context *g_usb_ctx = NULL;

/* Aux tracking struct for wispy1 characteristics */
typedef struct _wispy1_usb_aux {
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

	/* Calibration modes.  We avoid a "want calibration" by setting it to true */
	int calibrated;
	/* Calibration offset */
	int calibration_mod;
	/* Only allocated during calibration */
	int8_t *calibrationbuf;

	int sockpair[2];

	spectool_phy *phydev;
} wispy1_usb_aux;

/* Prototypes */
int wispy1_usb_open(spectool_phy *);
int wispy1_usb_close(spectool_phy *);
int wispy1_usb_poll(spectool_phy *);
int wispy1_usb_getpollfd(spectool_phy *);
void wispy1_usb_setcalibration(spectool_phy *, int);
spectool_sample_sweep *wispy1_usb_getsweep(spectool_phy *);

uint32_t wispy1_adler_checksum(const char *buf1, int len) {
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
int wispy1_usb_device_scan(spectool_device_list *list) {
	libusb_device **devlist;
	struct libusb_device_descriptor desc;
	ssize_t cnt, i;
	int num_found = 0;
	wispy1_usb_pair *auxpair;

	if (!g_usb_ctx)
		libusb_init(&g_usb_ctx);

	cnt = libusb_get_device_list(g_usb_ctx, &devlist);
	if (cnt < 0)
		return 0;

	for (i = 0; i < cnt; i++) {
		if (libusb_get_device_descriptor(devlist[i], &desc) != 0)
			continue;

		if (((desc.idVendor == METAGEEK_WISPY1A_VID) &&
			 (desc.idProduct == METAGEEK_WISPY1A_PID)) ||
			((desc.idVendor == METAGEEK_WISPY1B_VID) &&
			 (desc.idProduct == METAGEEK_WISPY1B_PID))) {

			if (list->num_devs == list->max_devs - 1)
				break;

			auxpair = (wispy1_usb_pair *) malloc(sizeof(wispy1_usb_pair));
			auxpair->bus      = libusb_get_bus_number(devlist[i]);
			auxpair->dev_addr = libusb_get_device_address(devlist[i]);

			uint8_t id_buf[2] = { auxpair->bus, auxpair->dev_addr };

			list->list[list->num_devs].device_id =
				wispy1_adler_checksum((char *)id_buf, 2);
			snprintf(list->list[list->num_devs].name, SPECTOOL_PHY_NAME_MAX,
					 "Wi-Spy v1 USB %u", list->list[list->num_devs].device_id);
			list->list[list->num_devs].init_func = wispy1_usb_init;
			list->list[list->num_devs].hw_rec = auxpair;

			list->list[list->num_devs].num_sweep_ranges = 1;

			list->list[list->num_devs].supported_ranges =
				(spectool_sample_sweep *) malloc(SPECTOOL_SWEEP_SIZE(0));

			list->list[list->num_devs].supported_ranges[0].name =
				strdup("2.4GHz ISM");
			list->list[list->num_devs].supported_ranges[0].start_khz = 2400000;
			list->list[list->num_devs].supported_ranges[0].end_khz = 2484000;
			list->list[list->num_devs].supported_ranges[0].res_hz = 1000 * 1000;
			list->list[list->num_devs].supported_ranges[0].num_samples =
				WISPY1_USB_NUM_SAMPLES;

			list->list[list->num_devs].supported_ranges[0].amp_offset_mdbm =
				WISPY1_USB_OFFSET_MDBM;
			list->list[list->num_devs].supported_ranges[0].amp_res_mdbm =
				WISPY1_USB_RES_MDBM;
			list->list[list->num_devs].supported_ranges[0].rssi_max =
				WISPY1_USB_RSSI_MAX;

			list->num_devs++;
			num_found++;
		}
	}

	libusb_free_device_list(devlist, 1);
	return num_found;
}

int wispy1_usb_init(spectool_phy *phydev, spectool_device_rec *rec) {
	wispy1_usb_pair *auxpair = (wispy1_usb_pair *) rec->hw_rec;

	if (auxpair == NULL)
		return -1;

	return wispy1_usb_init_path(phydev, auxpair->bus, auxpair->dev_addr);
}

/* Initialize a specific USB device based on bus number and device address */
int wispy1_usb_init_path(spectool_phy *phydev, uint8_t bus, uint8_t dev_addr) {
	libusb_device **devlist;
	libusb_device *found = NULL;
	struct libusb_device_descriptor desc;
	ssize_t cnt, i;
	uint8_t id_buf[2] = { bus, dev_addr };
	uint32_t cid;

	wispy1_usb_aux *auxptr = NULL;

	if (!g_usb_ctx)
		libusb_init(&g_usb_ctx);

	cid = wispy1_adler_checksum((char *)id_buf, 2);

	cnt = libusb_get_device_list(g_usb_ctx, &devlist);
	if (cnt < 0) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "WISPY1_INIT failed to enumerate USB devices");
		return -1;
	}

	for (i = 0; i < cnt; i++) {
		if (libusb_get_bus_number(devlist[i]) != bus)
			continue;
		if (libusb_get_device_address(devlist[i]) != dev_addr)
			continue;
		if (libusb_get_device_descriptor(devlist[i], &desc) != 0)
			continue;
		if (((desc.idVendor == METAGEEK_WISPY1A_VID) &&
			 (desc.idProduct == METAGEEK_WISPY1A_PID)) ||
			((desc.idVendor == METAGEEK_WISPY1B_VID) &&
			 (desc.idProduct == METAGEEK_WISPY1B_PID))) {
			found = libusb_ref_device(devlist[i]);
			break;
		} else {
			snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
					 "WISPY1_INIT failed, specified device %u does not "
					 "appear to be a Wi-Spy device", cid);
			libusb_free_device_list(devlist, 1);
			return -1;
		}
	}

	libusb_free_device_list(devlist, 1);

	if (found == NULL) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "WISPY1_INIT failed, specified device %u does not appear "
				 "to exist.", cid);
		return -1;
	}

	/* Build the device record with one sweep capability */
	phydev->device_spec = (spectool_dev_spec *) malloc(sizeof(spectool_dev_spec));

	phydev->device_spec->device_id = cid;

	/* Default the name to the buspath */
	snprintf(phydev->device_spec->device_name, SPECTOOL_PHY_NAME_MAX,
			 "Wi-Spy v1 USB %u", cid);

	/* State */
	phydev->state = SPECTOOL_STATE_CLOSED;

	phydev->min_rssi_seen = -1;

	phydev->device_spec->device_version = 0x01;
	phydev->device_spec->num_sweep_ranges = 1;

	phydev->device_spec->supported_ranges = 
		(spectool_sample_sweep *) malloc(SPECTOOL_SWEEP_SIZE(0));

	/* 2400 to 2484 MHz at 1MHz res */
	phydev->device_spec->supported_ranges[0].start_khz = 2400000;
	phydev->device_spec->supported_ranges[0].end_khz = 2484000;
	phydev->device_spec->supported_ranges[0].res_hz = 1000 * 1000;
	phydev->device_spec->supported_ranges[0].num_samples = WISPY1_USB_NUM_SAMPLES;

	phydev->device_spec->supported_ranges[0].amp_offset_mdbm = WISPY1_USB_OFFSET_MDBM;
	phydev->device_spec->supported_ranges[0].amp_res_mdbm = WISPY1_USB_RES_MDBM;
	phydev->device_spec->supported_ranges[0].rssi_max = WISPY1_USB_RSSI_MAX;

	/* Copy it into our default range data */
	phydev->device_spec->default_range = &(phydev->device_spec->supported_ranges[0]);

	phydev->device_spec->cur_profile = 0;

	/* Set up the aux state */
	auxptr = malloc(sizeof(wispy1_usb_aux));
	phydev->auxptr = auxptr;

	auxptr->configured = 0;

	auxptr->dev = found;
	auxptr->devhdl = NULL;
	auxptr->phydev = phydev;
	auxptr->sockpair[0] = -1;
	auxptr->sockpair[1] = -1;
	auxptr->sweepbuf_initialized = 0;

	auxptr->sweepbuf = 
		(spectool_sample_sweep *) malloc(SPECTOOL_SWEEP_SIZE(WISPY1_USB_NUM_SAMPLES));
	auxptr->sweepbuf->start_khz = 2400000;
	auxptr->sweepbuf->end_khz = 2484000;
	auxptr->sweepbuf->res_hz = 1000 * 1000;
	auxptr->sweepbuf->num_samples = WISPY1_USB_NUM_SAMPLES;
	auxptr->sweepbuf->amp_offset_mdbm = WISPY1_USB_OFFSET_MDBM;
	auxptr->sweepbuf->amp_res_mdbm = WISPY1_USB_RES_MDBM;
	auxptr->sweepbuf->rssi_max = WISPY1_USB_RSSI_MAX;

	auxptr->sweepbuf->phydev = phydev;

	/* init to -1 since the first pass at slot 0 increments */
	auxptr->num_sweeps = -1;
	/* Default to no calibration */
	auxptr->calibrated = 1;
	auxptr->calibration_mod = 0;
	auxptr->calibrationbuf = NULL;

	phydev->open_func = &wispy1_usb_open;
	phydev->close_func = &wispy1_usb_close;
	phydev->poll_func = &wispy1_usb_poll;
	phydev->pollfd_func = &wispy1_usb_getpollfd;
	phydev->setcalib_func = &wispy1_usb_setcalibration;
	phydev->getsweep_func = &wispy1_usb_getsweep;
	phydev->setposition_func = NULL;

	phydev->draw_agg_suggestion = 5;

	return 0;
}

void *wispy1_usb_servicethread(void *aux) {
	wispy1_usb_aux *auxptr = (wispy1_usb_aux *) aux;

	int sock;
	libusb_device_handle *wispy;

	char buf[8];
	struct timeval tm;
	int x = 0, error = 0;
	fd_set wset;

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
					 "wispy1_usb poller failed on IPC write select(): %s",
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
			int ret;
			buf[0] = (char) 0xFF;

			/* grab a HID control */
			ret = libusb_control_transfer(wispy,
					LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
					HID_GET_REPORT, (HID_RT_FEATURE << 8),
					0, (uint8_t *)buf, 8, TIMEOUT);
			if (ret < 0) {
				snprintf(auxptr->phydev->errstr, SPECTOOL_ERROR_MAX,
						 "wispy1_usb poller failed on libusb_control_transfer "
						 "HID cmd: %s", libusb_error_name(ret));
				auxptr->usb_thread_alive = 0;
				auxptr->phydev->state = SPECTOOL_STATE_ERROR;
				pthread_exit(NULL);
			}

			if (buf[0] == (char) 0xFF) {
				snprintf(auxptr->phydev->errstr, SPECTOOL_ERROR_MAX,
						 "wispy1_usb poller failed on usb_control_msg "
						 "HID cmd, no data returned, was the device removed?");
				auxptr->usb_thread_alive = 0;
				send(sock, buf, 8, 0);
				auxptr->phydev->state = SPECTOOL_STATE_ERROR;
				pthread_exit(NULL);
			}
		}

		/* Send it to the IPC remote, re-queue on enobufs */
		if (send(sock, buf, 8, 0) < 0) {
			if (errno == ENOBUFS) {
				error = 1;
				continue;
			}

			snprintf(auxptr->phydev->errstr, SPECTOOL_ERROR_MAX,
					 "wispy1_usb poller failed on IPC send: %s",
					 strerror(errno));
			auxptr->usb_thread_alive = 0;
			auxptr->phydev->state = SPECTOOL_STATE_ERROR;
			pthread_exit(NULL);
		}

		/* Flush and use select as a usleep to wait before reading from USB again */
		error = 0;
		tm.tv_sec = 0;
		tm.tv_usec = 7100;
		select(0, NULL, NULL, NULL, &tm);
	}

	auxptr->usb_thread_alive = 0;
	send(sock, buf, 8, 0);
	auxptr->phydev->state = SPECTOOL_STATE_ERROR;
	pthread_exit(NULL);
}

int wispy1_usb_getpollfd(spectool_phy *phydev) {
	wispy1_usb_aux *auxptr = (wispy1_usb_aux *) phydev->auxptr;

	if (auxptr->usb_thread_alive == 0) {
		wispy1_usb_close(phydev);
		return -1;
	}

	return auxptr->sockpair[0];
}

int wispy1_usb_open(spectool_phy *phydev) {
	int pid_status;
	wispy1_usb_aux *auxptr = (wispy1_usb_aux *) phydev->auxptr;

	/* Make the client/server socketpair */
	if (socketpair(PF_UNIX, SOCK_DGRAM, 0, auxptr->sockpair) < 0) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy1_usb open failed to create socket pair for capture "
				 "process: %s", strerror(errno));
		return -1;
	}

	int ret;
	ret = libusb_open(auxptr->dev, &auxptr->devhdl);
	if (ret != LIBUSB_SUCCESS) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy1_usb capture process failed to open USB device: %s",
				 libusb_error_name(ret));
		return -1;
	}

	ret = libusb_detach_kernel_driver(auxptr->devhdl, 0);
	if (ret != LIBUSB_SUCCESS && ret != LIBUSB_ERROR_NOT_FOUND &&
		ret != LIBUSB_ERROR_NOT_SUPPORTED) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy1_usb could not detach kernel driver: %s",
				 libusb_error_name(ret));
	}

	libusb_set_configuration(auxptr->devhdl, 1);

	ret = libusb_claim_interface(auxptr->devhdl, 0);
	if (ret != LIBUSB_SUCCESS) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy1_usb could not claim interface: %s",
				 libusb_error_name(ret));
		libusb_close(auxptr->devhdl);
		auxptr->devhdl = NULL;
		return -1;
	}

	auxptr->usb_thread_alive = 1;
	auxptr->last_read = time(0);

	if (pthread_create(&(auxptr->usb_thread), NULL, 
					   wispy1_usb_servicethread, auxptr) < 0) {
		snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
				 "wispy1_usb capture failed to create thread: %s",
				 strerror(errno));
		auxptr->usb_thread_alive = 0;
		return -1;
	}

	/* Update the state */
	phydev->state = SPECTOOL_STATE_CONFIGURING;

	return 1;
}

int wispy1_usb_close(spectool_phy *phydev) {
	wispy1_usb_aux *aux;
	
	if (phydev == NULL)
		return 0;

	aux = (wispy1_usb_aux *) phydev->auxptr;

	if (aux == NULL)
		return 0;

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

spectool_sample_sweep *wispy1_usb_getsweep(spectool_phy *phydev) {
	wispy1_usb_aux *auxptr = (wispy1_usb_aux *) phydev->auxptr;

	return auxptr->sweepbuf;
}

void wispy1_usb_setcalibration(spectool_phy *phydev, int in_calib) {
	wispy1_usb_aux *auxptr = (wispy1_usb_aux *) phydev->auxptr;
	int x;

	if (in_calib == 0 && auxptr->calibrationbuf != NULL) {
		free(auxptr->calibrationbuf);
		auxptr->calibrationbuf = NULL;
		auxptr->calibrated = 1;
		phydev->state = SPECTOOL_STATE_RUNNING;
	}

	if (in_calib != 0 && auxptr->calibrationbuf == NULL) {
		auxptr->calibrationbuf = 
			(int8_t *) malloc(sizeof(int8_t) * WISPY1_USB_NUM_SAMPLES);
		auxptr->calibrated = 0;
		for (x = 0; x < WISPY1_USB_NUM_SAMPLES; x++)
			auxptr->calibrationbuf[x] = 0;
		phydev->state = SPECTOOL_STATE_CALIBRATING;
	}
}

int wispy1_usb_poll(spectool_phy *phydev) {
	wispy1_usb_aux *auxptr = (wispy1_usb_aux *) phydev->auxptr;
	unsigned char lbuf[8];
	int x, pos, calfreqs, ret;
	long amptotal;
	int adjusted_rssi;

	/* Push a configure event before anything else */
	if (auxptr->configured == 0) {
		auxptr->configured = 1;
		return SPECTOOL_POLL_CONFIGURED;
	}

	/* Use the error set by the polling thread */
	if (auxptr->usb_thread_alive == 0) {
		phydev->state = SPECTOOL_STATE_ERROR;
		wispy1_usb_close(phydev);
		return SPECTOOL_POLL_ERROR;
	}

	if ((ret = recv(auxptr->sockpair[0], lbuf, 8, 0)) < 0) {
		if (auxptr->usb_thread_alive != 0)
			snprintf(phydev->errstr, SPECTOOL_ERROR_MAX,
					 "wispy1_usb IPC receiver failed to read signal data: %s",
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

	/* Initialize the sweep buffer when we get to it 
	 * If we haven't gotten around to a 0 state to initialize the buffer, we throw
	 * out the sample data until we do. */
	if (lbuf[0] == 0) {
		auxptr->sweepbuf_initialized = 1;
		auxptr->num_sweeps++;

		/* Init the timestamp for sweep begin */
		gettimeofday(&(auxptr->sweepbuf->tm_start), NULL);

		/* Catch the beginning of the sample loop and handle calibration if we've
		 * gathered enough data */
		if (auxptr->calibrated == 0 && 
			auxptr->num_sweeps >= WISPY1_USB_CALIBRATE_SWEEPS) {
			amptotal = 0;
			calfreqs = 0;

			/* Average all the signals below the calibration threshold to get a 
			 * level of fuzz to remove.  */
			for (x = 0; x < WISPY1_USB_NUM_SAMPLES; x++) {
				if (auxptr->calibrationbuf[x] < WISPY1_USB_RSSI_CALIBRATE_THRESH) {
					amptotal += auxptr->calibrationbuf[x];
					calfreqs++;
				}
			}

			if (calfreqs != 0) {
				auxptr->calibration_mod =
					(float) (((float) amptotal / (float) calfreqs) /
							 (float) WISPY1_USB_CALIBRATE_SWEEPS);
			}

			if (auxptr->calibration_mod > WISPY1_USB_CALIBRATE_MAX)
				auxptr->calibration_mod = WISPY1_USB_CALIBRATE_MAX;

			/* We're calibrated, destroy the buffer and flag us */
			auxptr->calibrated = 1;
			free(auxptr->calibrationbuf);
			auxptr->calibrationbuf = NULL;

			phydev->state = SPECTOOL_STATE_RUNNING;
		}

	} else if (auxptr->sweepbuf_initialized == 0) {
		return SPECTOOL_POLL_NONE;
	}

	for (x = 0; x < 7; x++) {
		/* Adjust the RSSI by the calibration */
		if (lbuf[x+1] > auxptr->calibration_mod)
			adjusted_rssi = lbuf[x+1] - auxptr->calibration_mod;
		else
			adjusted_rssi = lbuf[x+1];
		
		pos = lbuf[0] + x;

		if (pos >= WISPY1_USB_NUM_SAMPLES) {
			continue;
		}

		/* Assume the buffer exists, and write into the calibration record if it's a 
		 * maximum */
		if (auxptr->calibrated == 0 && auxptr->calibrationbuf != NULL) {
			if (adjusted_rssi > auxptr->calibrationbuf[pos])
				auxptr->calibrationbuf[pos] = adjusted_rssi;
		} else {
			/* write it into the main buffer */
			auxptr->sweepbuf->sample_data[pos] = adjusted_rssi;
		}

		if (phydev->min_rssi_seen > adjusted_rssi)
			phydev->min_rssi_seen = adjusted_rssi;

	}

	/* Flag that a sweep is complete */
	if (lbuf[0] + 7 >= WISPY1_USB_NUM_SAMPLES &&
		auxptr->calibrated) {
		gettimeofday(&(auxptr->sweepbuf->tm_end), NULL);
		auxptr->sweepbuf->min_rssi_seen = phydev->min_rssi_seen;
		return SPECTOOL_POLL_SWEEPCOMPLETE;
	}

	return SPECTOOL_POLL_NONE;
}

