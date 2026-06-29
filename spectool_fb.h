/*
 * Framebuffer frontend for spectrum-tools
 * Renders spectrum data directly to /dev/fb0 (Linux fbdev)
 *
 * This code is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef SPECTOOL_FB_H
#define SPECTOOL_FB_H

#ifdef HAVE_FB

#include <linux/fb.h>
#include <stdint.h>

#include "spectool_container.h"

typedef struct _spectool_fb_context {
	int fd;
	struct fb_var_screeninfo vinfo;
	struct fb_fix_screeninfo finfo;
	uint8_t *fb;
	size_t fb_size;
	int width, height, bpp;
	uint16_t *peak_hold;
} spectool_fb_context;

int spectool_fb_open(spectool_fb_context *ctx, const char *fbdev);
void spectool_fb_close(spectool_fb_context *ctx);
void spectool_fb_clear(spectool_fb_context *ctx);
void spectool_fb_draw_sweep(spectool_fb_context *ctx,
                             spectool_sample_sweep *sweep,
                             int amp_offset_mdbm, int amp_res_mdbm);
uint32_t spectool_fb_pixel(spectool_fb_context *ctx, uint8_t r, uint8_t g, uint8_t b);
void spectool_fb_putpixel(spectool_fb_context *ctx, int x, int y, uint32_t color);

#endif /* HAVE_FB */
#endif /* SPECTOOL_FB_H */
