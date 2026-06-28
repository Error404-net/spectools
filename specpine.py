#!/usr/bin/env python3
"""SpecPine v1.0 — RF spectrum analyzer for Hak5 WiFi Pineapple Pager.

Physical FB: 222×480 portrait  →  logical: 480×222 landscape.
Rotation: fb_offset = (lx * 222 + (221 - ly)) * 2
IPC: shell writes button commands to FIFO_PATH; Python reads non-blocking.
Data: spectools_bridge.py subprocess → JSONL events file → tailed here.
"""
from __future__ import annotations
import fcntl, json, os, select, signal, subprocess, sys, time
from collections import deque
from pathlib import Path

# ── Paths ──────────────────────────────────────────────────────────────────────
PAYLOAD_ROOT = Path("/mmc/root/payloads/user/reconnaissance/specpine")
SPECTOOL_BIN = str(PAYLOAD_ROOT / "bin/spectool_raw")
BRIDGE_BIN   = str(PAYLOAD_ROOT / "bin/spectools_bridge.py")
FB_PATH      = "/dev/fb0"
FIFO_PATH    = "/tmp/specpine_cmd.fifo"
EVENTS_PATH  = "/tmp/specpine_events.jsonl"
LOG_PATH     = "/tmp/specpine.log"
BUZZER_ROOT  = Path("/sys/devices/platform/buzzer_pwm/leds/buzzer")
VERSION      = "1.0"
AUTHOR       = "error404.net"

# ── Hardware ───────────────────────────────────────────────────────────────────
FB_W, FB_H   = 222, 480        # physical framebuffer (portrait)
IMG_W, IMG_H = 480, 222        # logical display (landscape)
BPP          = 2               # bytes per pixel (RGB565)

# ── Palette ────────────────────────────────────────────────────────────────────
C_BG      = (  0,   8,   4)
C_BAR     = (  0,  20,  10)
C_GREEN   = (  0, 220, 100)
C_GREEN_D = (  0, 140,  60)
C_GREEN_DD= (  0,  70,  30)
C_AMBER   = (255, 180,  40)
C_RED     = (220,  50,  60)
C_CYAN    = ( 60, 220, 220)
C_WHITE   = (220, 255, 220)
C_GRAY    = (  0,  80,  40)
C_MAGENTA = (255, 100, 200)

# ── Layout zones (logical y, origin=top-left) ──────────────────────────────────
TITLE_Y0,  TITLE_Y1  =   0,  14
TICK_Y0,   TICK_Y1   =  15,  19
WFALL_Y0,  WFALL_Y1  =  20, 181
LEGEND_Y0, LEGEND_Y1 = 182, 191
FREQ_Y0,   FREQ_Y1   = 192, 204
STATUS_Y0, STATUS_Y1 = 205, 221
WFALL_ROWS = WFALL_Y1 - WFALL_Y0 + 1   # 162

# Topo/Spectral share waterfall zone plus legend+freq for the plot area
PLOT_Y0, PLOT_Y1 = WFALL_Y0, FREQ_Y1   # 20..204

# ── Spectrum heat-map gradient ─────────────────────────────────────────────────
_GRADIENT = [
    (-100, (  0,   0,   0)),
    ( -88, (  0,   0, 160)),
    ( -78, (  0, 120, 220)),
    ( -70, (  0, 220,  80)),
    ( -60, (200, 220,   0)),
    ( -50, (255, 140,   0)),
    ( -38, (255,  20,   0)),
    (   0, (255, 180, 220)),
]

# ── Bitmap font 5×7, bit4=leftmost column ──────────────────────────────────────
_F: dict[str, list[int]] = {
    ' ':[0x00,0x00,0x00,0x00,0x00,0x00,0x00],
    '!':[0x04,0x04,0x04,0x04,0x04,0x00,0x04],
    '"':[0x0A,0x0A,0x00,0x00,0x00,0x00,0x00],
    '#':[0x0A,0x1F,0x0A,0x0A,0x1F,0x0A,0x00],
    "'":[0x04,0x04,0x00,0x00,0x00,0x00,0x00],
    '(':[0x02,0x04,0x08,0x08,0x08,0x04,0x02],
    ')':[0x08,0x04,0x02,0x02,0x02,0x04,0x08],
    '+':[0x00,0x04,0x04,0x1F,0x04,0x04,0x00],
    ',':[0x00,0x00,0x00,0x00,0x06,0x04,0x08],
    '-':[0x00,0x00,0x00,0x0E,0x00,0x00,0x00],
    '.':[0x00,0x00,0x00,0x00,0x00,0x06,0x06],
    '/':[0x01,0x02,0x02,0x04,0x08,0x10,0x10],
    '0':[0x0E,0x11,0x13,0x15,0x19,0x11,0x0E],
    '1':[0x04,0x0C,0x04,0x04,0x04,0x04,0x0E],
    '2':[0x0E,0x11,0x01,0x06,0x08,0x10,0x1F],
    '3':[0x1F,0x02,0x04,0x02,0x01,0x11,0x0E],
    '4':[0x02,0x06,0x0A,0x12,0x1F,0x02,0x02],
    '5':[0x1F,0x10,0x1E,0x01,0x01,0x11,0x0E],
    '6':[0x06,0x08,0x10,0x1E,0x11,0x11,0x0E],
    '7':[0x1F,0x01,0x02,0x04,0x08,0x08,0x08],
    '8':[0x0E,0x11,0x11,0x0E,0x11,0x11,0x0E],
    '9':[0x0E,0x11,0x11,0x0F,0x01,0x01,0x06],
    ':':[0x00,0x06,0x06,0x00,0x06,0x06,0x00],
    ';':[0x00,0x06,0x06,0x00,0x06,0x04,0x08],
    '<':[0x02,0x04,0x08,0x10,0x08,0x04,0x02],
    '=':[0x00,0x00,0x1F,0x00,0x1F,0x00,0x00],
    '>':[0x08,0x04,0x02,0x01,0x02,0x04,0x08],
    '?':[0x0E,0x11,0x01,0x06,0x04,0x00,0x04],
    'A':[0x04,0x0A,0x11,0x11,0x1F,0x11,0x11],
    'B':[0x1E,0x11,0x11,0x1E,0x11,0x11,0x1E],
    'C':[0x0E,0x11,0x10,0x10,0x10,0x11,0x0E],
    'D':[0x1C,0x12,0x11,0x11,0x11,0x12,0x1C],
    'E':[0x1F,0x10,0x10,0x1E,0x10,0x10,0x1F],
    'F':[0x1F,0x10,0x10,0x1E,0x10,0x10,0x10],
    'G':[0x0E,0x11,0x10,0x13,0x11,0x11,0x0E],
    'H':[0x11,0x11,0x11,0x1F,0x11,0x11,0x11],
    'I':[0x0E,0x04,0x04,0x04,0x04,0x04,0x0E],
    'J':[0x0F,0x01,0x01,0x01,0x01,0x11,0x0E],
    'K':[0x11,0x12,0x14,0x18,0x14,0x12,0x11],
    'L':[0x10,0x10,0x10,0x10,0x10,0x10,0x1F],
    'M':[0x11,0x1B,0x15,0x15,0x11,0x11,0x11],
    'N':[0x11,0x19,0x15,0x13,0x11,0x11,0x11],
    'O':[0x0E,0x11,0x11,0x11,0x11,0x11,0x0E],
    'P':[0x1E,0x11,0x11,0x1E,0x10,0x10,0x10],
    'Q':[0x0E,0x11,0x11,0x11,0x15,0x12,0x0D],
    'R':[0x1E,0x11,0x11,0x1E,0x14,0x12,0x11],
    'S':[0x0E,0x11,0x10,0x0E,0x01,0x11,0x0E],
    'T':[0x1F,0x04,0x04,0x04,0x04,0x04,0x04],
    'U':[0x11,0x11,0x11,0x11,0x11,0x11,0x0E],
    'V':[0x11,0x11,0x11,0x11,0x0A,0x0A,0x04],
    'W':[0x11,0x11,0x11,0x15,0x1B,0x11,0x11],
    'X':[0x11,0x0A,0x04,0x04,0x04,0x0A,0x11],
    'Y':[0x11,0x11,0x0A,0x04,0x04,0x04,0x04],
    'Z':[0x1F,0x01,0x02,0x04,0x08,0x10,0x1F],
    '[':[0x06,0x04,0x04,0x04,0x04,0x04,0x06],
    ']':[0x06,0x02,0x02,0x02,0x02,0x02,0x06],
    '_':[0x00,0x00,0x00,0x00,0x00,0x00,0x1F],
    'a':[0x00,0x00,0x0E,0x01,0x0F,0x11,0x0F],
    'b':[0x10,0x10,0x1E,0x11,0x11,0x11,0x1E],
    'c':[0x00,0x00,0x0E,0x11,0x10,0x11,0x0E],
    'd':[0x01,0x01,0x0F,0x11,0x11,0x11,0x0F],
    'e':[0x00,0x00,0x0E,0x11,0x1F,0x10,0x0E],
    'f':[0x06,0x09,0x08,0x1E,0x08,0x08,0x08],
    'g':[0x00,0x0F,0x11,0x11,0x0F,0x01,0x0E],
    'h':[0x10,0x10,0x16,0x19,0x11,0x11,0x11],
    'i':[0x04,0x00,0x0C,0x04,0x04,0x04,0x0E],
    'j':[0x01,0x00,0x03,0x01,0x01,0x11,0x0E],
    'k':[0x10,0x10,0x12,0x14,0x18,0x14,0x12],
    'l':[0x0C,0x04,0x04,0x04,0x04,0x04,0x0E],
    'm':[0x00,0x00,0x11,0x1B,0x15,0x11,0x11],
    'n':[0x00,0x00,0x16,0x19,0x11,0x11,0x11],
    'o':[0x00,0x00,0x0E,0x11,0x11,0x11,0x0E],
    'p':[0x00,0x00,0x1E,0x11,0x1E,0x10,0x10],
    'q':[0x00,0x00,0x0F,0x11,0x0F,0x01,0x01],
    'r':[0x00,0x00,0x16,0x19,0x10,0x10,0x10],
    's':[0x00,0x00,0x0E,0x10,0x0E,0x01,0x0E],
    't':[0x04,0x04,0x1F,0x04,0x04,0x04,0x03],
    'u':[0x00,0x00,0x11,0x11,0x11,0x11,0x0F],
    'v':[0x00,0x00,0x11,0x11,0x0A,0x0A,0x04],
    'w':[0x00,0x00,0x11,0x11,0x15,0x1B,0x11],
    'x':[0x00,0x00,0x11,0x0A,0x04,0x0A,0x11],
    'y':[0x00,0x00,0x11,0x11,0x0F,0x01,0x0E],
    'z':[0x00,0x00,0x1F,0x02,0x04,0x08,0x1F],
    '|':[0x04,0x04,0x04,0x04,0x04,0x04,0x04],
    '~':[0x00,0x08,0x15,0x02,0x00,0x00,0x00],
}

# ── FB primitives ──────────────────────────────────────────────────────────────

def _rgb(r: int, g: int, b: int) -> int:
    return ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)

def _put(fb: bytearray, lx: int, ly: int, px: int) -> None:
    off = (lx * FB_W + (FB_H - 1 - ly)) * BPP
    fb[off] = px & 0xFF;  fb[off + 1] = px >> 8

def _fill(fb: bytearray, x: int, y: int, w: int, h: int, px: int) -> None:
    lo, hi = px & 0xFF, px >> 8
    tile = bytes([lo, hi]) * h
    col0 = FB_H - 1 - (y + h - 1)
    for lx in range(x, x + w):
        base = (lx * FB_W + col0) * BPP
        fb[base: base + h * BPP] = tile

def _hline(fb: bytearray, x0: int, x1: int, ly: int, px: int) -> None:
    lo, hi = px & 0xFF, px >> 8
    col = FB_H - 1 - ly
    for lx in range(x0, x1 + 1):
        off = (lx * FB_W + col) * BPP
        fb[off] = lo;  fb[off + 1] = hi

def _vline(fb: bytearray, lx: int, y0: int, y1: int, px: int) -> None:
    lo, hi = px & 0xFF, px >> 8
    for ly in range(y0, y1 + 1):
        off = (lx * FB_W + (FB_H - 1 - ly)) * BPP
        fb[off] = lo;  fb[off + 1] = hi

def _vline_dot(fb: bytearray, lx: int, y0: int, y1: int, px: int) -> None:
    lo, hi = px & 0xFF, px >> 8
    for ly in range(y0, y1 + 1, 2):
        off = (lx * FB_W + (FB_H - 1 - ly)) * BPP
        fb[off] = lo;  fb[off + 1] = hi

def _glyph(fb: bytearray, ch: str, cx: int, cy: int, px: int, scale: int = 1) -> None:
    rows = _F.get(ch, _F[' '])
    for ri, bits in enumerate(rows):
        for ci in range(5):
            if bits & (1 << (4 - ci)):
                for sy in range(scale):
                    for sx in range(scale):
                        _put(fb, cx + ci * scale + sx, cy + ri * scale + sy, px)

def _text(fb: bytearray, s: str, x: int, y: int, px: int, scale: int = 1) -> int:
    stride = (5 * scale) + scale
    for i, ch in enumerate(s):
        _glyph(fb, ch, x + i * stride, y, px, scale)
    return x + len(s) * stride

def _text_center(fb: bytearray, s: str, y: int, px: int, scale: int = 1) -> None:
    stride = (5 * scale) + scale
    x = (IMG_W - len(s) * stride) // 2
    _text(fb, s, max(0, x), y, px, scale)

def _box(fb: bytearray, x: int, y: int, w: int, h: int, px: int) -> None:
    _hline(fb, x, x + w - 1, y, px)
    _hline(fb, x, x + w - 1, y + h - 1, px)
    _vline(fb, x, y, y + h - 1, px)
    _vline(fb, x + w - 1, y, y + h - 1, px)

def flush_fb(fb: bytearray) -> None:
    try:
        with open(FB_PATH, "r+b", buffering=0) as dev:
            dev.seek(0); dev.write(fb)
    except OSError as e:
        _log(f"fb write: {e}")

# ── Pineapple UI suspend ───────────────────────────────────────────────────────
_pine_pid: int | None = None

def _find_pine() -> int | None:
    for entry in os.listdir("/proc"):
        if not entry.isdigit(): continue
        try:
            comm = Path(f"/proc/{entry}/comm").read_text().strip()
            if comm == "pineapple": return int(entry)
        except OSError: pass
    return None

def pine_stop() -> None:
    global _pine_pid
    _pine_pid = _find_pine()
    if _pine_pid:
        try: os.kill(_pine_pid, signal.SIGSTOP)
        except OSError: pass

def pine_cont() -> None:
    if _pine_pid:
        try: os.kill(_pine_pid, signal.SIGCONT)
        except OSError: pass

# ── Buzzer / RTTTL ────────────────────────────────────────────────────────────
_NOTE_HZ = {'c':261.63,'d':293.66,'e':329.63,'f':349.23,
            'g':392.00,'a':440.00,'b':493.88,'p':0.0}
_SEMI = 2.0 ** (1/12)

def _note_hz(note: str, octave: int, sharp: bool) -> float:
    base = _NOTE_HZ.get(note, 0.0)
    if base == 0.0: return 0.0
    base *= 2.0 ** (octave - 4)
    return base * (_SEMI if sharp else 1.0)

def _parse_rtttl(s: str) -> list[tuple[float, float]]:
    parts = s.split(':', 2)
    if len(parts) < 3: return []
    cfg = {k.strip(): v.strip() for p in parts[1].split(',') if '=' in p for k,v in [p.split('=',1)]}
    d_dur = int(cfg.get('d', 4)); d_oct = int(cfg.get('o', 5)); bpm = int(cfg.get('b', 120))
    qn = 60.0 / bpm
    out = []
    for tok in parts[2].split(','):
        tok = tok.strip().lower()
        if not tok: continue
        i = 0
        dur = d_dur
        while i < len(tok) and tok[i].isdigit(): i += 1
        if i > 0: dur = int(tok[:i])
        if i >= len(tok): continue
        note = tok[i]; i += 1
        sharp = i < len(tok) and tok[i] == '#'
        if sharp: i += 1
        oct_ = d_oct
        if i < len(tok) and tok[i].isdigit(): oct_ = int(tok[i]); i += 1
        dot = i < len(tok) and tok[i] == '.'
        sec = qn * 4 / dur * (1.5 if dot else 1.0)
        out.append((_note_hz(note, oct_, sharp), sec))
    return out

def _buz(freq: float, on: bool, vol: int = 60) -> None:
    try:
        if on and freq > 0:
            (BUZZER_ROOT / "frequency").write_text(str(int(freq)))
            (BUZZER_ROOT / "volume").write_text(str(vol))
        (BUZZER_ROOT / "brightness").write_text("1" if (on and freq > 0) else "0")
    except OSError: pass

def play_tone(rtttl: str, vol: int = 55) -> None:
    for freq, dur in _parse_rtttl(rtttl):
        _buz(freq, True, vol)
        time.sleep(dur * 0.82)
        _buz(0, False)
        time.sleep(dur * 0.18)

def vibrate(ms: int = 120) -> None:
    """Short vibration burst via low-frequency buzzer."""
    try:
        (BUZZER_ROOT / "frequency").write_text("80")
        (BUZZER_ROOT / "volume").write_text("100")
        (BUZZER_ROOT / "brightness").write_text("1")
        time.sleep(ms / 1000.0)
        (BUZZER_ROOT / "brightness").write_text("0")
    except OSError: pass

# RTTTL tones — original retro-keygen / demo-scene compositions
TONE_STARTUP    = "SpecBoot:d=16,o=5,b=140:c,e,g,c6,e6,4g6"
TONE_NAV        = "Nav:d=32,o=5,b=200:g,b"
TONE_DEVICE_OK  = "Found:d=8,o=5,b=160:g,b,4e6"
TONE_SCREENSHOT = "Snap:d=16,o=6,b=220:c,p,c,e"
TONE_ERROR      = "Err:d=8,o=5,b=110:b,4f"
TONE_EXIT       = "Bye:d=8,o=4,b=95:8c5,8g4,8e4,4c4"
TONE_SELECT     = "Sel:d=32,o=6,b=180:c,e"

# ── Spectrum LUT ───────────────────────────────────────────────────────────────
def _build_lut() -> list[int]:
    lut: list[int] = []
    for dbm in range(-128, 128):
        r = g = b = 0
        if dbm <= _GRADIENT[0][0]: r,g,b = _GRADIENT[0][1]
        elif dbm >= _GRADIENT[-1][0]: r,g,b = _GRADIENT[-1][1]
        else:
            for i in range(len(_GRADIENT)-1):
                ld,lc = _GRADIENT[i]; hd,hc = _GRADIENT[i+1]
                if ld <= dbm <= hd:
                    t = (dbm-ld)/(hd-ld)
                    r=int(lc[0]+t*(hc[0]-lc[0])); g=int(lc[1]+t*(hc[1]-lc[1])); b=int(lc[2]+t*(hc[2]-lc[2]))
                    break
        lut.append(_rgb(r,g,b))
    return lut

def _lut(lut: list[int], dbm: float) -> int:
    return lut[max(0, min(255, int(round(dbm)) + 128))]

# ── Wi-Fi channel maps ─────────────────────────────────────────────────────────
_CH_24 = {1:2412, 6:2437, 11:2462}
_CH_5  = {36:5180,40:5200,44:5220,48:5240,149:5745,153:5765,157:5785,161:5805}

def _freq_x(mhz: float, s: float, e: float) -> int:
    return int((mhz - s) / (e - s) * (IMG_W - 1)) if e > s else 0

# ── Shared static frame (title bar + channel ticks + legend + freq labels) ─────
def draw_static(fb: bytearray, label: str, freq_start_khz: int | None,
                freq_end_khz: int | None, view_hint: str = "") -> None:
    bg  = _rgb(*C_BG);  bar = _rgb(*C_BAR)
    grn = _rgb(*C_GREEN); gray = _rgb(*C_GRAY)
    amb = _rgb(*C_AMBER); wht = _rgb(*C_WHITE)

    _fill(fb, 0, 0, IMG_W, IMG_H, bg)
    _fill(fb, 0, TITLE_Y0, IMG_W, TITLE_Y1 - TITLE_Y0 + 1, bar)
    _text(fb, "SpecPine", 4, TITLE_Y0 + 4, _rgb(*C_GREEN))
    _text(fb, label, 76, TITLE_Y0 + 4, amb)
    if view_hint:
        hint_x = IMG_W - len(view_hint) * 6 - 2
        _text(fb, view_hint, hint_x, TITLE_Y0 + 4, gray)
    _hline(fb, 0, IMG_W - 1, TITLE_Y1, grn)

    s_mhz = (freq_start_khz or 2400000) / 1000.0
    e_mhz = (freq_end_khz   or 2483500) / 1000.0
    channels = _CH_5 if (freq_start_khz or 0) >= 3_000_000 else _CH_24
    for ch, ch_mhz in channels.items():
        if not (s_mhz <= ch_mhz <= e_mhz): continue
        cx = _freq_x(ch_mhz, s_mhz, e_mhz)
        _fill(fb, cx-1, TICK_Y0, 3, TICK_Y1-TICK_Y0+1, grn)
        _vline_dot(fb, cx, WFALL_Y0, WFALL_Y1, gray)
        _text(fb, str(ch), max(0, cx - len(str(ch))*3), FREQ_Y0 + 5, grn)

    for lx in range(IMG_W):
        dbm = int(-100 + lx/(IMG_W-1)*80)
        r2=g2=b2=0
        if dbm <= _GRADIENT[0][0]: r2,g2,b2 = _GRADIENT[0][1]
        elif dbm >= _GRADIENT[-1][0]: r2,g2,b2 = _GRADIENT[-1][1]
        else:
            for i in range(len(_GRADIENT)-1):
                ld,lc=_GRADIENT[i]; hd,hc=_GRADIENT[i+1]
                if ld<=dbm<=hd:
                    t=(dbm-ld)/(hd-ld)
                    r2=int(lc[0]+t*(hc[0]-lc[0])); g2=int(lc[1]+t*(hc[1]-lc[1])); b2=int(lc[2]+t*(hc[2]-lc[2]))
                    break
        c2 = _rgb(r2,g2,b2)
        for ly in range(LEGEND_Y0, LEGEND_Y1+1): _put(fb, lx, ly, c2)
    for dv in (-95,-80,-70,-60,-50,-40):
        lx = int((dv-(-100))/80*(IMG_W-1))
        _text(fb, str(dv), lx, LEGEND_Y1+1, gray)

    if freq_start_khz and freq_end_khz:
        _text(fb, f"{int(s_mhz)}MHz", 0, FREQ_Y0, gray)
        es = f"{int(e_mhz)}MHz"
        _text(fb, es, IMG_W - len(es)*6 - 2, FREQ_Y0, gray)

    _fill(fb, 0, STATUS_Y0, IMG_W, STATUS_Y1-STATUS_Y0+1, bar)
    _hline(fb, 0, IMG_W-1, STATUS_Y0, grn)

def draw_status(fb: bytearray, sweeps: int, peak: int | None,
                state: str, band: str, paused: bool) -> None:
    bar = _rgb(*C_BAR); wht = _rgb(*C_WHITE)
    grn = _rgb(*C_GREEN); gray = _rgb(*C_GRAY); amb = _rgb(*C_AMBER)
    _fill(fb, 0, STATUS_Y0+1, IMG_W, STATUS_Y1-STATUS_Y0, bar)
    _text(fb, f"SWP:{sweeps}", 2, STATUS_Y0+5, wht)
    if peak is not None:
        _text(fb, f"PK:{peak}dBm", 90, STATUS_Y0+5, grn if peak < -60 else _rgb(*C_RED))
    _text(fb, band, 230, STATUS_Y0+5, amb)
    s = "PAUSED" if paused else state[:8]
    _text(fb, s, 340, STATUS_Y0+5, amb if paused else gray)
    _text(fb, "<L/R>views", 390, STATUS_Y0+5, gray)

# ── Loading screen ─────────────────────────────────────────────────────────────
def draw_loading(fb: bytearray, msg: str = "INITIALIZING...") -> None:
    bg = _rgb(*C_BG); grn = _rgb(*C_GREEN); bar = _rgb(*C_BAR)
    amb = _rgb(*C_AMBER); gray = _rgb(*C_GRAY)
    _fill(fb, 0, 0, IMG_W, IMG_H, bg)

    # Retro border
    _box(fb, 2, 2, IMG_W-4, IMG_H-4, _rgb(*C_GREEN_DD))
    _box(fb, 4, 4, IMG_W-8, IMG_H-8, grn)

    # Big title
    _text_center(fb, "SpecPine", 28, grn, scale=2)
    _text_center(fb, f"v{VERSION}", 50, _rgb(*C_GREEN_DD))
    _hline(fb, 20, IMG_W-20, 65, _rgb(*C_GREEN_DD))

    # Subtext
    _text_center(fb, "RF SPECTRUM ANALYZER", 74, amb)
    _text_center(fb, "PINEAPPLE PAGER EDITION", 86, gray)
    _hline(fb, 20, IMG_W-20, 100, _rgb(*C_GREEN_DD))

    # Status msg — blinks
    _text_center(fb, msg, 120, grn)

    # Boot art (CRT scan effect)
    for ly in range(140, 180, 4):
        _hline(fb, 20, IMG_W-20, ly, _rgb(0, 15, 7))

    _text_center(fb, "Wi-Spy DBx // libusb-1.0", 188, gray)
    _text_center(fb, AUTHOR, 200, _rgb(*C_GREEN_DD))

# ── Error screen ───────────────────────────────────────────────────────────────
def draw_error(fb: bytearray, title: str, detail: str) -> None:
    bg = _rgb(*C_BG); red = _rgb(*C_RED); gray = _rgb(*C_GRAY)
    _fill(fb, 0, 0, IMG_W, IMG_H, bg)
    _box(fb, 4, 4, IMG_W-8, IMG_H-8, red)
    _text_center(fb, "!! ERROR !!", 20, red)
    _hline(fb, 10, IMG_W-10, 34, red)
    _text_center(fb, title, 50, _rgb(*C_WHITE))
    # Word-wrap detail at 38 chars
    words = detail.split(); line = ""; y = 70
    for w in words:
        if len(line) + len(w) + 1 > 38:
            _text_center(fb, line.strip(), y, gray); y += 12; line = ""
        line += w + " "
    if line: _text_center(fb, line.strip(), y, gray)
    _text_center(fb, "BACK: return to menu", IMG_H - 16, gray)

# ── Main menu ──────────────────────────────────────────────────────────────────
MENU_ITEMS = [
    "WATERFALL VIEW",
    "SPECTRAL VIEW",
    "TOPO VIEW",
    "PLANAR VIEW",
    "DEVICE INFO",
    "ABOUT / NFO",
    "EXIT",
]

def draw_menu(fb: bytearray, sel: int) -> None:
    bg  = _rgb(*C_BG);  bar = _rgb(*C_BAR)
    grn = _rgb(*C_GREEN); gray = _rgb(*C_GRAY)
    amb = _rgb(*C_AMBER); wht = _rgb(*C_WHITE)
    _fill(fb, 0, 0, IMG_W, IMG_H, bg)

    # Header
    _fill(fb, 0, 0, IMG_W, 18, bar)
    _text(fb, "SpecPine", 4, 4, grn)
    _text(fb, f"v{VERSION}", 68, 4, _rgb(*C_GREEN_DD))
    _text(fb, "RF SPECTRUM ANALYZER", 140, 4, amb)
    _hline(fb, 0, IMG_W-1, 17, grn)

    # Decorative left column
    _vline(fb, 8, 22, IMG_H-22, _rgb(*C_GREEN_DD))
    _vline(fb, 9, 22, IMG_H-22, _rgb(*C_GREEN_DD))

    # Menu items
    item_h = 22
    start_y = 28
    visible = min(len(MENU_ITEMS), 8)
    for i in range(visible):
        y = start_y + i * item_h
        is_sel = i == sel
        if is_sel:
            _fill(fb, 14, y-1, IMG_W-20, item_h-2, _rgb(0, 40, 20))
            _hline(fb, 14, IMG_W-7, y-1, grn)
            _hline(fb, 14, IMG_W-7, y+item_h-3, grn)
            _text(fb, ">", 16, y+7, amb)
            _text(fb, f"[ {MENU_ITEMS[i]} ]", 28, y+7, wht)
        else:
            _text(fb, f"  {MENU_ITEMS[i]}", 28, y+7, gray)

    # Footer
    _hline(fb, 0, IMG_W-1, IMG_H-17, grn)
    _fill(fb, 0, IMG_H-16, IMG_W, 16, bar)
    _text(fb, "UP/DN:nav  OK:select  BACK:exit", 10, IMG_H-11, gray)

# ── Frequency selection ────────────────────────────────────────────────────────
BANDS = [("2.4 GHz", 0), ("5 GHz (full)", 2)]

def draw_freq_select(fb: bytearray, sel: int, menu_item: str) -> None:
    bg = _rgb(*C_BG); bar = _rgb(*C_BAR)
    grn = _rgb(*C_GREEN); gray = _rgb(*C_GRAY); amb = _rgb(*C_AMBER)
    _fill(fb, 0, 0, IMG_W, IMG_H, bg)
    _fill(fb, 0, 0, IMG_W, 18, bar)
    _text(fb, "SpecPine", 4, 4, grn)
    _text(fb, "SELECT BAND", 140, 4, amb)
    _hline(fb, 0, IMG_W-1, 17, grn)

    _text_center(fb, menu_item, 40, _rgb(*C_WHITE))
    _hline(fb, 40, IMG_W-40, 54, _rgb(*C_GREEN_DD))

    for i, (label, _) in enumerate(BANDS):
        y = 75 + i * 35
        if i == sel:
            _fill(fb, 60, y-2, IMG_W-120, 22, _rgb(0, 40, 20))
            _box(fb, 60, y-2, IMG_W-120, 22, grn)
            _text_center(fb, f"> {label} <", y+6, _rgb(*C_WHITE))
        else:
            _text_center(fb, label, y+6, gray)

    _hline(fb, 0, IMG_W-1, IMG_H-17, grn)
    _fill(fb, 0, IMG_H-16, IMG_W, 16, bar)
    _text(fb, "UP/DN:select  OK:start  BACK:menu", 6, IMG_H-11, gray)

# ── About / NFO screen ─────────────────────────────────────────────────────────
_NFO_LINES = [
    ("", C_BG),
    (" +-[ SpecPine v1.0 ]-----------------------------+", C_GREEN),
    (" |  RF SPECTRUM ANALYZER - PINEAPPLE PAGER ED.  |", C_WHITE),
    (" |  mipsel_24kc // musl // libusb-1.0           |", C_GRAY),
    (" +-----------------------------------------------+", C_GREEN),
    ("", C_BG),
    (" =-[ ABOUT ]=====================================", C_AMBER),
    ("  SpecPine brings the classic Spectools suite", C_WHITE),
    ("  to the Hak5 Pineapple Pager framebuffer.", C_WHITE),
    ("  Wi-Spy DBx scans RF; we paint the pixels.", C_WHITE),
    ("", C_BG),
    (" =-[ GREETS + CREDITS ]==========================", C_AMBER),
    ("  :: hak5 community", C_GREEN),
    ("  :: spectools / caljorden", C_GREEN),
    ("  :: spectools / dragorn", C_GREEN),
    ("  :: ArmoredPixie", C_GREEN),
    ("", C_BG),
    (" =-[ BASED ON ]==================================", C_AMBER),
    ("  spectrum-tools by kismetwireless", C_WHITE),
    ("  kismetwireless.net/code/spectools/", C_GRAY),
    ("  libusb-1.0 port :: error404.net", C_WHITE),
    ("", C_BG),
    (" =-[ LICENSE ]===================================", C_AMBER),
    ("  GNU GPL v2 -- see source for full terms.", C_GRAY),
    ("  built on OpenWrt 24.10.1 for mipsel_24kc", C_GRAY),
    ("", C_BG),
    (" =-[ BUILD ]=====================================", C_AMBER),
    ("  spectool_raw // libusb-1.0 static", C_WHITE),
    ("  spectools_bridge.py // JSONL pipeline", C_WHITE),
    ("  specpine.py // unified FB renderer", C_WHITE),
    ("", C_BG),
    (" +-----------------------------------------------+", C_GREEN_DD),
    (" |   NO CARRIER  //  0 FILES  //  0 DAYS        |", C_GRAY),
    (" +-----------------------------------------------+", C_GREEN_DD),
    ("", C_BG),
]
_NFO_LINE_H = 10  # px per line

def draw_about(fb: bytearray, scroll: int) -> None:
    bg = _rgb(*C_BG); bar = _rgb(*C_BAR); grn = _rgb(*C_GREEN)
    gray = _rgb(*C_GRAY); amb = _rgb(*C_AMBER)
    _fill(fb, 0, 0, IMG_W, IMG_H, bg)
    _fill(fb, 0, 0, IMG_W, 18, bar)
    _text(fb, "SpecPine", 4, 4, grn)
    _text(fb, "ABOUT / NFO", 140, 4, amb)
    _hline(fb, 0, IMG_W-1, 17, grn)

    content_y0, content_y1 = 20, IMG_H - 18
    visible_lines = (content_y1 - content_y0) // _NFO_LINE_H

    for i in range(visible_lines):
        idx = scroll + i
        if idx >= len(_NFO_LINES): break
        text, color = _NFO_LINES[idx]
        if text:
            _text(fb, text[:78], 2, content_y0 + i * _NFO_LINE_H, _rgb(*color))

    # Scrollbar
    total = len(_NFO_LINES)
    if total > visible_lines:
        bar_h = max(4, visible_lines * (content_y1-content_y0) // total)
        bar_y = content_y0 + scroll * (content_y1 - content_y0 - bar_h) // (total - visible_lines)
        _vline(fb, IMG_W-3, content_y0, content_y1, _rgb(*C_GREEN_DD))
        _vline(fb, IMG_W-3, bar_y, bar_y+bar_h, grn)

    _hline(fb, 0, IMG_W-1, IMG_H-17, grn)
    _fill(fb, 0, IMG_H-16, IMG_W, 16, bar)
    _text(fb, "UP/DN:scroll  BACK:menu", 10, IMG_H-11, gray)

# ── Device info screen ─────────────────────────────────────────────────────────
def draw_device_info(fb: bytearray, dev_name: str, freq_start: int | None,
                     freq_end: int | None, bin_count: int | None,
                     sweep_count: int, peak: int | None) -> None:
    bg = _rgb(*C_BG); bar = _rgb(*C_BAR); grn = _rgb(*C_GREEN)
    wht = _rgb(*C_WHITE); gray = _rgb(*C_GRAY); amb = _rgb(*C_AMBER)
    _fill(fb, 0, 0, IMG_W, IMG_H, bg)
    _fill(fb, 0, 0, IMG_W, 18, bar)
    _text(fb, "SpecPine", 4, 4, grn)
    _text(fb, "DEVICE INFO", 140, 4, amb)
    _hline(fb, 0, IMG_W-1, 17, grn)
    _box(fb, 10, 24, IMG_W-20, IMG_H-50, _rgb(*C_GREEN_DD))
    y = 34
    def row(label: str, val: str, col=wht):
        nonlocal y
        _text(fb, f"{label}:", 16, y, gray)
        _text(fb, val, 120, y, col)
        y += 14
    row("DEVICE", dev_name[:22] if dev_name else "unknown", grn)
    row("FREQ", f"{(freq_start or 0)//1000}-{(freq_end or 0)//1000} MHz", wht)
    row("BINS", str(bin_count or "--"))
    row("SWEEPS", str(sweep_count))
    row("PEAK", f"{peak} dBm" if peak else "--", _rgb(*C_RED) if (peak and peak > -50) else wht)
    row("SPECTOOL", "spectool_raw // libusb-1.0", gray)
    row("BUILD", f"specpine v{VERSION}", gray)
    _hline(fb, 0, IMG_W-1, IMG_H-17, grn)
    _fill(fb, 0, IMG_H-16, IMG_W, 16, bar)
    _text(fb, "BACK: return", 10, IMG_H-11, gray)

# ── Waterfall view ─────────────────────────────────────────────────────────────
def sweep_to_row(bins: list[int], lut: list[int]) -> list[int]:
    n = len(bins)
    if n >= IMG_W:
        chunk = n / IMG_W
        return [_lut(lut, max(bins[int(i*chunk):max(int((i+1)*chunk), int(i*chunk)+1)])) for i in range(IMG_W)]
    return [_lut(lut, bins[min(int(i*n/IMG_W), n-1)]) for i in range(IMG_W)]

def draw_waterfall_sweep(fb: bytearray, row: list[int], ring_len: int) -> None:
    base_col = FB_H - 1 - WFALL_Y1
    n_after = min(ring_len, WFALL_ROWS)
    shift_n = n_after - 1
    for lx in range(IMG_W):
        off = (lx * FB_W + base_col) * BPP
        if shift_n > 0:
            fb[off + BPP: off + BPP + shift_n * BPP] = fb[off: off + shift_n * BPP]
        px = row[lx]
        fb[off] = px & 0xFF;  fb[off + 1] = px >> 8

# ── Spectral view (live bar chart) ────────────────────────────────────────────
_spectral_peak: list[int] = []

def reset_spectral() -> None:
    global _spectral_peak
    _spectral_peak = []

def draw_spectral(fb: bytearray, bins: list[int], lut: list[int],
                  freq_start_khz: int | None, freq_end_khz: int | None) -> None:
    global _spectral_peak
    if not _spectral_peak or len(_spectral_peak) != IMG_W:
        _spectral_peak = [-128] * IMG_W

    plot_h = WFALL_Y1 - WFALL_Y0 + 1 + (LEGEND_Y1 - LEGEND_Y0 + 1) + (FREQ_Y1 - FREQ_Y0 + 1)
    plot_h = FREQ_Y1 - WFALL_Y0 + 1   # full plot area height
    bg = _rgb(*C_BG); grn = _rgb(*C_GREEN); gray = _rgb(*C_GRAY)
    wht_pk = _rgb(*C_WHITE)

    _fill(fb, 0, WFALL_Y0, IMG_W, plot_h, bg)
    n = len(bins)
    for lx in range(IMG_W):
        idx = int(lx * n / IMG_W) if n < IMG_W else int(lx * n / IMG_W)
        idx = min(idx, n-1)
        dbm = bins[idx]
        pct = max(0, min(plot_h-1, int((dbm - (-100)) / 70.0 * (plot_h-1))))
        bar_y0 = FREQ_Y1 - pct;  bar_y1 = FREQ_Y1
        px = _lut(lut, dbm)
        _vline(fb, lx, bar_y0, bar_y1, px)
        # Peak hold
        if dbm > _spectral_peak[lx]:
            _spectral_peak[lx] = dbm
        pk_pct = max(0, min(plot_h-1, int((_spectral_peak[lx] - (-100)) / 70.0 * (plot_h-1))))
        _put(fb, lx, FREQ_Y1 - pk_pct, wht_pk)

# ── Topo view (2D persistence heatmap) ────────────────────────────────────────
_topo_buf: list[list[int]] | None = None
_TOPO_DECAY = 1

def reset_topo() -> None:
    global _topo_buf
    plot_h = FREQ_Y1 - WFALL_Y0 + 1
    _topo_buf = [[0] * IMG_W for _ in range(plot_h)]

def draw_topo(fb: bytearray, bins: list[int]) -> None:
    global _topo_buf
    plot_h = FREQ_Y1 - WFALL_Y0 + 1
    if _topo_buf is None or len(_topo_buf) != plot_h:
        reset_topo()
    assert _topo_buf is not None

    n = len(bins)
    # Decay
    for row in _topo_buf:
        for lx in range(IMG_W):
            if row[lx] > 0: row[lx] = max(0, row[lx] - _TOPO_DECAY)

    # Paint current sweep
    for lx in range(IMG_W):
        idx = min(int(lx * n / IMG_W), n-1)
        dbm = bins[idx]
        pct = max(0, min(plot_h-1, int((dbm - (-100)) / 70.0 * (plot_h-1))))
        _topo_buf[pct][lx] = min(255, _topo_buf[pct][lx] + 8)

    # Render
    bg = _rgb(*C_BG)
    for iy in range(plot_h):
        ly = WFALL_Y0 + iy
        for lx in range(IMG_W):
            v = _topo_buf[iy][lx]
            if v == 0:
                _put(fb, lx, ly, bg)
            else:
                # Map intensity (0-255) to heat color
                r = min(255, v * 2)
                g = min(255, max(0, v * 3 - 200))
                b = max(0, 100 - v)
                _put(fb, lx, ly, _rgb(r, g, b))

# ── Planar view (current + avg + peak lines) ──────────────────────────────────
_planar_avg: list[float] = []
_planar_peak: list[int] = []
_PLANAR_AVG_W = 0.2   # EMA weight

def reset_planar() -> None:
    global _planar_avg, _planar_peak
    _planar_avg = [];  _planar_peak = []

def draw_planar(fb: bytearray, bins: list[int], lut: list[int]) -> None:
    global _planar_avg, _planar_peak
    plot_h = FREQ_Y1 - WFALL_Y0 + 1
    bg  = _rgb(*C_BG);  grn = _rgb(*C_GREEN)
    amb = _rgb(*C_AMBER); wht = _rgb(*C_WHITE)
    red = _rgb(*C_RED);   gray = _rgb(*C_GRAY)
    _fill(fb, 0, WFALL_Y0, IMG_W, plot_h, bg)

    n = len(bins)
    if not _planar_avg: _planar_avg = [float(bins[min(int(i*n/IMG_W),n-1)]) for i in range(IMG_W)]
    if not _planar_peak: _planar_peak = list(bins[:IMG_W]) if n >= IMG_W else [bins[min(int(i*n/IMG_W),n-1)] for i in range(IMG_W)]

    def dbm_y(dbm: float) -> int:
        pct = max(0, min(plot_h-1, int((dbm-(-100))/70.0*(plot_h-1))))
        return FREQ_Y1 - pct

    # Subtle grid lines at -80, -60, -40 dBm
    for gdbm in (-80, -60, -40):
        gy = dbm_y(gdbm)
        _hline(fb, 0, IMG_W-1, gy, _rgb(*C_GREEN_DD))

    prev_c = prev_a = prev_p = None
    for lx in range(IMG_W):
        idx = min(int(lx*n/IMG_W), n-1)
        cur = bins[idx]
        _planar_avg[lx] = _planar_avg[lx]*(1-_PLANAR_AVG_W) + cur*_PLANAR_AVG_W
        if cur > _planar_peak[lx]: _planar_peak[lx] = cur

        yc = dbm_y(cur);  ya = dbm_y(_planar_avg[lx]);  yp = dbm_y(_planar_peak[lx])
        _put(fb, lx, yc, grn)
        _put(fb, lx, ya, amb)
        _put(fb, lx, yp, red)
        # connect lines vertically for density
        if prev_c is not None:
            for ly in range(min(prev_c,yc), max(prev_c,yc)+1): _put(fb, lx, ly, grn)
        prev_c = yc

    # Legend
    ly = WFALL_Y0 + 2
    _text(fb, "CUR", IMG_W-28, ly, grn)
    _text(fb, "AVG", IMG_W-28, ly+10, amb)
    _text(fb, "PK", IMG_W-28, ly+20, red)

# ── Screenshot ─────────────────────────────────────────────────────────────────
def take_screenshot(fb: bytearray) -> str | None:
    ts = time.strftime("%Y%m%d_%H%M%S")
    loot = Path("/root/loot/specpine")
    loot.mkdir(parents=True, exist_ok=True)
    # Find/create session dir
    sessions = sorted(loot.glob("session_*"))
    sess = sessions[-1] if sessions else loot / f"session_{ts}_fb"
    sess.mkdir(exist_ok=True)
    path = sess / f"screenshot_{ts}.raw"
    try:
        with open(FB_PATH, "rb") as f:
            data = f.read()
        path.write_bytes(data)
        # Flash effect
        white_buf = bytearray(b'\xff\xff' * (FB_W * FB_H))
        with open(FB_PATH, "r+b", buffering=0) as dev:
            dev.seek(0); dev.write(white_buf)
        time.sleep(0.12)
        flush_fb(fb)
        return str(path)
    except OSError as e:
        _log(f"screenshot: {e}")
        return None

# ── Logging ────────────────────────────────────────────────────────────────────
def _log(msg: str) -> None:
    try:
        with open(LOG_PATH, "a") as f:
            f.write(f"[specpine] {time.strftime('%H:%M:%S')} {msg}\n")
    except OSError: pass

# ── FIFO helpers ───────────────────────────────────────────────────────────────
def open_fifo_nb(path: str) -> int:
    if not os.path.exists(path):
        os.mkfifo(path)
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    return fd

def read_cmd(fd: int) -> str | None:
    try:
        data = os.read(fd, 64)
        return data.decode().strip().split('\n')[-1] if data else None
    except BlockingIOError:
        return None
    except OSError:
        return None

# ── Bridge subprocess ──────────────────────────────────────────────────────────
def start_bridge(range_idx: int) -> subprocess.Popen:
    cmd = f"LD_LIBRARY_PATH={PAYLOAD_ROOT}/lib {SPECTOOL_BIN} --range {range_idx}"
    return subprocess.Popen(
        ["python3", BRIDGE_BIN, "--input-command", cmd,
         "--events-file", EVENTS_PATH, "--follow",
         "--stall-timeout", "6", "--max-restarts", "5"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )

# ── App state machine ──────────────────────────────────────────────────────────
VIEWS = ['waterfall', 'spectral', 'topo', 'planar']
VIEW_LABELS = {'waterfall':'WATERFALL', 'spectral':'SPECTRAL', 'topo':'TOPO', 'planar':'PLANAR'}

class SpecPine:
    def __init__(self):
        self.fb       = bytearray(FB_W * FB_H * BPP)
        self.lut      = _build_lut()
        self.screen   = 'loading'      # loading|menu|freq_select|view|about|devinfo|error
        self.menu_sel = 0
        self.freq_sel = 0              # index into BANDS
        self.view_idx = 0              # index into VIEWS
        self.paused   = False
        self.running  = True

        # Sweep data
        self.ring: deque[list[int]] = deque(maxlen=WFALL_ROWS)
        self.current_bins: list[int] = []
        self.sweep_count = 0
        self.peak: int | None = None
        self.freq_start: int | None = None
        self.freq_end:   int | None = None
        self.dev_name   = ""
        self.bin_count: int | None = None

        # View state
        self.about_scroll = 0
        self.static_dirty = True       # needs draw_static() refresh

        # Processes
        self.bridge: subprocess.Popen | None = None
        self.events_fp = None
        self.fifo_fd: int | None = None

    @property
    def view(self) -> str:
        return VIEWS[self.view_idx]

    @property
    def band_label(self) -> str:
        return BANDS[self.freq_sel][0]

    def start(self):
        pine_stop()
        # Setup FIFO
        try:
            self.fifo_fd = open_fifo_nb(FIFO_PATH)
        except OSError as e:
            _log(f"fifo: {e}")

        # Loading screen + startup tone (non-blocking: play in bg)
        draw_loading(self.fb, "LOADING...")
        flush_fb(self.fb)

        import threading
        threading.Thread(target=play_tone, args=(TONE_STARTUP,), daemon=True).start()

        # Device detection
        draw_loading(self.fb, "DETECTING DEVICE...")
        flush_fb(self.fb)
        time.sleep(0.3)

        try:
            out = subprocess.check_output(
                [SPECTOOL_BIN, "-l"], timeout=5,
                stderr=subprocess.STDOUT, text=True
            )
            if "Wi-Spy" not in out and "wispy" not in out.lower():
                raise RuntimeError("No Wi-Spy device found")
            _log(f"device ok: {out.splitlines()[0]}")
        except Exception as e:
            self.screen = 'error'
            self._error_title = "DEVICE NOT FOUND"
            self._error_detail = str(e)
            play_tone(TONE_ERROR)
            return

        draw_loading(self.fb, "DEVICE OK!")
        flush_fb(self.fb)
        play_tone(TONE_DEVICE_OK)
        time.sleep(0.4)

        self.screen = 'menu'

    def open_events(self):
        if self.events_fp:
            try: self.events_fp.close()
            except: pass
        try:
            Path(EVENTS_PATH).unlink(missing_ok=True)
            self.events_fp = open(EVENTS_PATH, "r", encoding="utf-8")
            # seek to end so we only see fresh events
        except OSError: pass

    def start_scan(self):
        if self.bridge:
            self.bridge.terminate()
            try: self.bridge.wait(timeout=2)
            except: self.bridge.kill()
        range_idx = BANDS[self.freq_sel][1]
        self.ring.clear(); self.current_bins = []
        self.sweep_count = 0; self.peak = None
        self.freq_start = self.freq_end = None
        self.static_dirty = True
        reset_spectral(); reset_topo(); reset_planar()
        self.open_events()
        self.bridge = start_bridge(range_idx)
        _log(f"bridge started range={range_idx}")

    def stop_scan(self):
        if self.bridge:
            self.bridge.terminate()
            try: self.bridge.wait(timeout=2)
            except: self.bridge.kill()
            self.bridge = None
        if self.events_fp:
            try: self.events_fp.close()
            except: pass
            self.events_fp = None

    def process_event_line(self, raw: str):
        try: evt = json.loads(raw)
        except: return
        t = evt.get("type")
        if t == "device_config":
            if evt.get("freq_start_khz"): self.freq_start = evt["freq_start_khz"]
            if evt.get("freq_end_khz"):   self.freq_end   = evt["freq_end_khz"]
            if evt.get("bin_count"):      self.bin_count  = evt["bin_count"]
            if evt.get("device_name"):    self.dev_name   = evt["device_name"]
            self.static_dirty = True
        elif t == "sweep" and not self.paused:
            if evt.get("freq_start_khz"): self.freq_start = evt["freq_start_khz"]
            if evt.get("freq_end_khz"):   self.freq_end   = evt["freq_end_khz"]
            bins = evt.get("rssi_bins", [])
            if bins:
                self.current_bins = bins
                self.sweep_count += 1
                self.peak = max(bins)
                if self.view == 'waterfall':
                    row = sweep_to_row(bins, self.lut)
                    self.ring.appendleft(row)
                    draw_waterfall_sweep(self.fb, row, len(self.ring))

    def handle_cmd(self, cmd: str):
        _log(f"cmd={cmd} screen={self.screen}")
        if self.screen == 'menu':
            if cmd == 'UP':
                self.menu_sel = (self.menu_sel - 1) % len(MENU_ITEMS)
                play_tone(TONE_NAV)
            elif cmd == 'DOWN':
                self.menu_sel = (self.menu_sel + 1) % len(MENU_ITEMS)
                play_tone(TONE_NAV)
            elif cmd == 'OK':
                play_tone(TONE_SELECT)
                self._menu_select()
            elif cmd == 'BACK':
                play_tone(TONE_EXIT)
                self.running = False

        elif self.screen == 'freq_select':
            if cmd == 'UP':
                self.freq_sel = (self.freq_sel - 1) % len(BANDS)
                play_tone(TONE_NAV)
            elif cmd == 'DOWN':
                self.freq_sel = (self.freq_sel + 1) % len(BANDS)
                play_tone(TONE_NAV)
            elif cmd == 'OK':
                play_tone(TONE_SELECT)
                self.start_scan()
                self.screen = 'view'
                self.static_dirty = True
            elif cmd == 'BACK':
                play_tone(TONE_NAV)
                self.screen = 'menu'

        elif self.screen == 'view':
            if cmd == 'LEFT':
                self.view_idx = (self.view_idx - 1) % len(VIEWS)
                self.static_dirty = True; play_tone(TONE_NAV)
            elif cmd == 'RIGHT':
                self.view_idx = (self.view_idx + 1) % len(VIEWS)
                self.static_dirty = True; play_tone(TONE_NAV)
            elif cmd == 'OK':
                self.paused = not self.paused
                play_tone(TONE_NAV)
            elif cmd == 'BACK':
                play_tone(TONE_NAV)
                self.stop_scan()
                self.screen = 'menu'
            elif cmd == 'SCREENSHOT':
                path = take_screenshot(self.fb)
                vibrate(150)
                play_tone(TONE_SCREENSHOT)
                if path: _log(f"screenshot: {path}")

        elif self.screen in ('about', 'devinfo', 'error'):
            if cmd == 'UP' and self.screen == 'about':
                self.about_scroll = max(0, self.about_scroll - 2)
            elif cmd == 'DOWN' and self.screen == 'about':
                self.about_scroll = min(len(_NFO_LINES)-1, self.about_scroll + 2)
            elif cmd == 'BACK':
                play_tone(TONE_NAV)
                self.screen = 'menu'

    def _menu_select(self):
        idx = self.menu_sel
        if idx == 0:   # Waterfall
            self.view_idx = 0; self.screen = 'freq_select'
        elif idx == 1:  # Spectral
            self.view_idx = 1; self.screen = 'freq_select'
        elif idx == 2:  # Topo
            self.view_idx = 2; self.screen = 'freq_select'
        elif idx == 3:  # Planar
            self.view_idx = 3; self.screen = 'freq_select'
        elif idx == 4:  # Device info
            self.screen = 'devinfo'
        elif idx == 5:  # About
            self.about_scroll = 0; self.screen = 'about'
        elif idx == 6:  # Exit
            play_tone(TONE_EXIT)
            self.running = False

    def render(self):
        s = self.screen
        if s == 'loading':
            pass  # already drawn in start()
        elif s == 'menu':
            draw_menu(self.fb, self.menu_sel)
            flush_fb(self.fb)
        elif s == 'freq_select':
            draw_freq_select(self.fb, self.freq_sel, MENU_ITEMS[self.menu_sel])
            flush_fb(self.fb)
        elif s == 'view':
            if self.static_dirty:
                draw_static(self.fb, VIEW_LABELS[self.view],
                            self.freq_start, self.freq_end,
                            f"<{self.view.upper()}>")
                self.static_dirty = False
            v = self.view
            if v in ('spectral', 'topo', 'planar') and self.current_bins:
                if v == 'spectral':
                    draw_spectral(self.fb, self.current_bins, self.lut,
                                  self.freq_start, self.freq_end)
                elif v == 'topo':
                    draw_topo(self.fb, self.current_bins)
                elif v == 'planar':
                    draw_planar(self.fb, self.current_bins, self.lut)
            draw_status(self.fb, self.sweep_count, self.peak,
                        "PAUSED" if self.paused else "SCANNING",
                        self.band_label, self.paused)
            if self.paused:
                # Translucent "PAUSED" overlay in the middle
                px = (IMG_W - 7*12) // 2; py = IMG_H//2 - 8
                _fill(self.fb, px-4, py-4, 7*12+8, 20, _rgb(0,40,20))
                _box(self.fb, px-4, py-4, 7*12+8, 20, _rgb(*C_AMBER))
                _text(self.fb, "PAUSED", px, py+2, _rgb(*C_AMBER))
            flush_fb(self.fb)
        elif s == 'about':
            draw_about(self.fb, self.about_scroll)
            flush_fb(self.fb)
        elif s == 'devinfo':
            draw_device_info(self.fb, self.dev_name, self.freq_start, self.freq_end,
                             self.bin_count, self.sweep_count, self.peak)
            flush_fb(self.fb)
        elif s == 'error':
            draw_error(self.fb, getattr(self, '_error_title', 'ERROR'),
                       getattr(self, '_error_detail', ''))
            flush_fb(self.fb)

    def run(self):
        self.start()
        if not self.running:
            self.render(); flush_fb(self.fb); time.sleep(3)
            return

        # Track which screens need to be re-rendered on each tick
        last_screen = None
        last_sweep  = -1
        last_sel    = -1
        frame_t     = 1.0 / 8   # 8fps cap

        while self.running:
            t0 = time.time()

            # Read FIFO command
            if self.fifo_fd is not None:
                cmd = read_cmd(self.fifo_fd)
                if cmd:
                    self.handle_cmd(cmd)

            # Read bridge events (non-blocking tail)
            if self.events_fp:
                for _ in range(50):   # drain up to 50 lines per tick
                    line = self.events_fp.readline()
                    if not line: break
                    self.process_event_line(line.strip())

            # Render if something changed
            needs_render = (
                self.screen != last_screen or
                self.sweep_count != last_sweep or
                self.menu_sel != last_sel or
                self.static_dirty
            )
            if needs_render:
                self.render()
                last_screen = self.screen
                last_sweep  = self.sweep_count
                last_sel    = self.menu_sel

            elapsed = time.time() - t0
            if elapsed < frame_t:
                time.sleep(frame_t - elapsed)

    def shutdown(self):
        self.stop_scan()
        if self.fifo_fd is not None:
            try: os.close(self.fifo_fd)
            except: pass
        pine_cont()
        _log("shutdown")

def main():
    app = SpecPine()
    def _sig(*_):
        app.running = False
    signal.signal(signal.SIGTERM, _sig)
    signal.signal(signal.SIGINT, _sig)
    try:
        app.run()
    finally:
        app.shutdown()

if __name__ == "__main__":
    main()
