/*
 * Framebuffer frontend for spectrum-tools
 * Renders spectrum data directly to /dev/fb0 for the Hak5 Pineapple Pager LCD.
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "config.h"

#ifdef HAVE_FB

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/types.h>
#include <linux/fb.h>
#include <libusb.h>

#include "spectool_container.h"
#include "spectool_net_client.h"
#include "spectool_fb.h"

static spectool_phy *g_devs = NULL;
static spectool_fb_context g_fb;
static volatile int g_running = 1;

int spectool_fb_open(spectool_fb_context *ctx, const char *fbdev) {
	ctx->fd = open(fbdev ? fbdev : "/dev/fb0", O_RDWR);
	if (ctx->fd < 0)
		return -1;

	if (ioctl(ctx->fd, FBIOGET_VSCREENINFO, &ctx->vinfo) < 0 ||
		ioctl(ctx->fd, FBIOGET_FSCREENINFO, &ctx->finfo) < 0) {
		close(ctx->fd);
		return -1;
	}

	ctx->width   = ctx->vinfo.xres;
	ctx->height  = ctx->vinfo.yres;
	ctx->bpp     = ctx->vinfo.bits_per_pixel;
	ctx->fb_size = ctx->finfo.smem_len;

	ctx->fb = mmap(NULL, ctx->fb_size, PROT_READ | PROT_WRITE, MAP_SHARED, ctx->fd, 0);
	if (ctx->fb == MAP_FAILED) {
		close(ctx->fd);
		return -1;
	}

	ctx->peak_hold = calloc(ctx->width, sizeof(uint16_t));
	if (!ctx->peak_hold) {
		munmap(ctx->fb, ctx->fb_size);
		close(ctx->fd);
		return -1;
	}

	return 0;
}

void spectool_fb_close(spectool_fb_context *ctx) {
	if (ctx->peak_hold) {
		free(ctx->peak_hold);
		ctx->peak_hold = NULL;
	}
	if (ctx->fb && ctx->fb != MAP_FAILED) {
		munmap(ctx->fb, ctx->fb_size);
		ctx->fb = NULL;
	}
	if (ctx->fd >= 0) {
		close(ctx->fd);
		ctx->fd = -1;
	}
}

uint32_t spectool_fb_pixel(spectool_fb_context *ctx, uint8_t r, uint8_t g, uint8_t b) {
	if (ctx->bpp == 16)
		return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
	return ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
}

void spectool_fb_putpixel(spectool_fb_context *ctx, int x, int y, uint32_t color) {
	size_t off;

	if (x < 0 || x >= ctx->width || y < 0 || y >= ctx->height)
		return;

	off = (size_t)(y * ctx->finfo.line_length) + (size_t)(x * (ctx->bpp / 8));
	if (ctx->bpp == 16)
		*(uint16_t *)(ctx->fb + off) = (uint16_t)color;
	else
		*(uint32_t *)(ctx->fb + off) = color;
}

void spectool_fb_clear(spectool_fb_context *ctx) {
	memset(ctx->fb, 0, ctx->fb_size);
}

void spectool_fb_draw_sweep(spectool_fb_context *ctx,
							 spectool_sample_sweep *sweep,
							 int amp_offset_mdbm, int amp_res_mdbm) {
	int s, x, y;
	int samples = sweep->num_samples;

	spectool_fb_clear(ctx);

	for (s = 0; s < samples; s++) {
		int amp_mdbm = amp_offset_mdbm + (sweep->sample_data[s] * amp_res_mdbm);
		/* Map -100 dBm to -30 dBm → 0 to ctx->height */
		int range_mdbm = 70 * 1000;
		int base_mdbm  = -100 * 1000;
		int pct = (amp_mdbm - base_mdbm) * ctx->height / range_mdbm;

		if (pct < 0)         pct = 0;
		if (pct > ctx->height) pct = ctx->height;

		x = (s * ctx->width) / samples;

		uint32_t color;
		if (pct > ctx->height * 2 / 3)
			color = spectool_fb_pixel(ctx, 255, 0, 0);   /* red: strong */
		else if (pct > ctx->height / 3)
			color = spectool_fb_pixel(ctx, 255, 200, 0); /* yellow: medium */
		else
			color = spectool_fb_pixel(ctx, 0, 200, 0);   /* green: weak */

		for (y = ctx->height - pct; y < ctx->height; y++)
			spectool_fb_putpixel(ctx, x, y, color);

		if (pct > ctx->peak_hold[x])
			ctx->peak_hold[x] = (uint16_t)pct;

		spectool_fb_putpixel(ctx, x, ctx->height - ctx->peak_hold[x],
							  spectool_fb_pixel(ctx, 255, 255, 255));
	}
}

static void sighandle(int sig) {
	(void)sig;
	g_running = 0;
}

static void Usage(void) {
	printf("spectool_fb [ options ]\n"
		   " -d / --device <idx>    Use device index (default: 0)\n"
		   " -f / --fb <fbdev>      Framebuffer device (default: /dev/fb0)\n"
		   " -r / --range <range>   Select sweep range index\n"
		   " -l / --list            List devices and ranges, then exit\n"
		   " -h / --help            This help\n");
}

int main(int argc, char *argv[]) {
	spectool_device_list list;
	spectool_phy *pi;
	spectool_sample_sweep *sb;
	char errstr[SPECTOOL_ERROR_MAX];
	char *fbdev = NULL;
	int dev_idx = 0;
	int range_idx = 0;
	int list_only = 0;
	int ndev, x, ret;

	static struct option long_options[] = {
		{ "device",  required_argument, 0, 'd' },
		{ "fb",      required_argument, 0, 'f' },
		{ "range",   required_argument, 0, 'r' },
		{ "list",    no_argument,       0, 'l' },
		{ "help",    no_argument,       0, 'h' },
		{ 0, 0, 0, 0 }
	};
	int option_index;

	while (1) {
		int o = getopt_long(argc, argv, "d:f:r:lh", long_options, &option_index);
		if (o < 0) break;
		switch (o) {
		case 'd': dev_idx   = atoi(optarg); break;
		case 'f': fbdev     = optarg;       break;
		case 'r': range_idx = atoi(optarg); break;
		case 'l': list_only = 1;            break;
		case 'h': Usage(); return 0;
		}
	}

	ndev = spectool_device_scan(&list);

	if (ndev <= 0) {
		fprintf(stderr, "No spectool devices found\n");
		return 1;
	}

	if (list_only) {
		printf("Found %d device(s):\n", ndev);
		for (x = 0; x < ndev; x++) {
			printf("  [%d] %s (id %u)\n", x, list.list[x].name, list.list[x].device_id);
			for (int r = 0; r < list.list[x].num_sweep_ranges; r++) {
				spectool_sample_sweep *ran = &list.list[x].supported_ranges[r];
				printf("      range %d: \"%s\" %d kHz - %d kHz, %d samples\n",
					   r, ran->name, ran->start_khz, ran->end_khz, ran->num_samples);
			}
		}
		spectool_device_scan_free(&list);
		return 0;
	}

	if (dev_idx < 0 || dev_idx >= ndev) {
		fprintf(stderr, "Device index %d out of range (0-%d)\n", dev_idx, ndev - 1);
		return 1;
	}

	pi = (spectool_phy *) malloc(SPECTOOL_PHY_SIZE);
	memset(pi, 0, SPECTOOL_PHY_SIZE);
	g_devs = pi;

	if (spectool_device_init(pi, &list.list[dev_idx]) < 0) {
		fprintf(stderr, "Error initializing device: %s\n", spectool_get_error(pi));
		return 1;
	}

	if (spectool_phy_open(pi) < 0) {
		fprintf(stderr, "Error opening device: %s\n", spectool_get_error(pi));
		return 1;
	}

	spectool_phy_setcalibration(pi, 1);
	spectool_phy_setposition(pi, range_idx, 0, 0);
	spectool_device_scan_free(&list);

	if (spectool_fb_open(&g_fb, fbdev) < 0) {
		fprintf(stderr, "Failed to open framebuffer %s: %s\n",
				fbdev ? fbdev : "/dev/fb0", strerror(errno));
		return 1;
	}

	printf("Rendering to %s (%dx%d bpp=%d)\n",
		   fbdev ? fbdev : "/dev/fb0",
		   g_fb.width, g_fb.height, g_fb.bpp);

	signal(SIGINT,  sighandle);
	signal(SIGTERM, sighandle);

	while (g_running) {
		fd_set rfds;
		struct timeval tm;
		int pfd = spectool_phy_getpollfd(pi);

		if (pfd < 0) {
			fprintf(stderr, "Device error: %s\n", spectool_get_error(pi));
			break;
		}

		FD_ZERO(&rfds);
		FD_SET(pfd, &rfds);
		tm.tv_sec  = 0;
		tm.tv_usec = 100000;

		if (select(pfd + 1, &rfds, NULL, NULL, &tm) < 0) {
			if (errno == EINTR) continue;
			break;
		}

		if (!FD_ISSET(pfd, &rfds))
			continue;

		ret = spectool_phy_poll(pi);

		if (ret & SPECTOOL_POLL_ERROR) {
			fprintf(stderr, "Device error: %s\n", spectool_get_error(pi));
			break;
		}

		if (ret & SPECTOOL_POLL_SWEEPCOMPLETE) {
			sb = spectool_phy_getsweep(pi);
			if (sb) {
				spectool_fb_draw_sweep(&g_fb, sb,
									   sb->amp_offset_mdbm,
									   sb->amp_res_mdbm);
			}
		}
	}

	spectool_fb_close(&g_fb);
	spectool_phy_close(pi);
	free(pi);

	return 0;
}

#endif /* HAVE_FB */
