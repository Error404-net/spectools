# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Spectrum Tools — userspace drivers and utilities for MetaGeek Wi-Spy USB spectrum analyzer hardware, plus the Ubertooth U1. Implements four binaries: `spectool_raw`, `spectool_net`, `spectool_gtk`, and `spectool_curses`.

## Building

```bash
./configure        # detects GTK, libusb, ncurses; reports which targets will build
make               # builds configured targets
make clean         # removes .o files and binaries
make distclean     # also removes Makefile, config.h, config.log
sudo make install  # installs to /usr/local/bin (or configured prefix)
```

**Required deps:** libusb **0.1.x** (not 1.x — the configure check will reject 1.x; use the compatibility layer if needed), pthreads, libm.  
**Optional deps:** GTK2 + Cairo → builds `spectool_gtk`; ncurses/libcurses → builds `spectool_curses`.

On macOS, configure adds `-framework IOKit -framework CoreFoundation` automatically.

## Architecture

**Core abstraction** (`spectool_container.h`):
- `spectool_phy` — device handle with vtable function pointers (`open_func`, `close_func`, `poll_func`, `getsweep_func`, etc.). All hardware drivers fill this struct.
- `spectool_sample_sweep` — one sweep record: frequency range, RSSI conversion params, and a flexible `uint8_t sample_data[0]` array. Allocate with `SPECTOOL_SWEEP_SIZE(n)`.
- `spectool_sweep_cache` — ring buffer of sweeps, maintains computed `avg`, `peak`, and `roll_peak` sweeps. The GTK widgets consume this.
- RSSI → dBm conversion: `SPECTOOL_RSSI_CONVERT(amp_offset_mdbm, amp_res_mdbm, rssi)`.

**Hardware drivers** (each implements the `spectool_phy` interface):
- `wispy_hw_gen1` — original Wi-Spy (2.4 GHz only, 1 range)
- `wispy_hw_24x` — 2.4x series
- `wispy_hw_dbx` — DBx series
- `ubertooth_hw_u1` — Ubertooth U1

Device scan (`spectool_device_scan`) enumerates USB, returns a `spectool_device_list`; `spectool_device_init` fills a `spectool_phy` from a `spectool_device_rec`.

**Network layer** (`spectool_net.h`):
- `spectool_net_server.c` → `spectool_net` binary: exposes local devices over TCP, supports broadcast announcement and remote config.
- `spectool_net_client.c` → used by `spectool_raw`, `spectool_curses`, `spectool_gtk` to consume remote devices transparently alongside local USB devices.

**GTK UI** (`spectool_gtk_*.c`):
- `spectool_gtk_hw_registry` — tracks active `spectool_phy` devices for the UI.
- `spectool_gtk_widget` — base GTK widget that all graph types extend.
- `spectool_gtk_planar` — traditional SA view (current/avg/peak lines).
- `spectool_gtk_spectral` — waterfall view (color intensity = power over time).
- `spectool_gtk_topo` — 2D peak-frequency-over-time topographic view.
- `spectool_gtk_channel` — overlays 802.11 channel markers on graphs.

**Channel definitions** are static arrays in `spectool_container.h`: 2.4 GHz (802.11b/g, 14 ch), 5 GHz sub-bands (802.11a low/mid/high), 900 MHz ISM.

## Linux udev

Copy `99-wispy.rules` to `/etc/udev/rules.d/` to allow non-root access for users in the `plugdev` group. Restart udevd after installing.
