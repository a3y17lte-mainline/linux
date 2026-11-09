// SPDX-License-Identifier: GPL-2.0-only
/*
 * Samsung AMS474MM01 command mode MIPI DSI panel with S6E3AA2 controller,
 * found in the Samsung Galaxy A3 (2017).
 *
 * The brightness tables and the gamma calculation follow Samsung's
 * downstream driver for this panel.
 *
 * Copyright (C) 2026 Martin Štrobl <recordyletsplays@gmail.com>
 */

#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

#include "panel-samsung-dsi.h"

/* Manufacturer Command Set */
#define MCS_AIDCTL		0xb1	/* AMOLED impulse driving (AOR) */
#define MCS_ELVSS		0xb5
#define MCS_MTP			0xc8	/* factory gamma calibration */
#define MCS_GAMMACTL		0xca
#define MCS_PCD			0xcc
#define MCS_ERRFLAG		0xed
#define MCS_GAMMAUPD		0xf7

#define S6E3AA2_NUM_LEVELS	256
#define S6E3AA2_DEFAULT_LEVEL	128

/*
 * Gamma is set as reference voltages at 11 taps of the grey scale: VT
 * (the black level) and the grey levels listed below. Both the factory
 * calibration (MTP) and the gamma register use the same 35 byte layout.
 */
#define S6E3AA2_NUM_TAPS	11
#define S6E3AA2_TAP_VT		0
#define S6E3AA2_TAP_TOP		(S6E3AA2_NUM_TAPS - 1)
#define S6E3AA2_GAMMA_LEN	35

static const u8 s6e3aa2_tap_grey[S6E3AA2_NUM_TAPS] = {
	0, 1, 7, 11, 23, 35, 51, 87, 151, 203, 255
};

/* Reference voltage of the gamma generator, 6.0 V in units of 10 uV */
#define S6E3AA2_VREG		600000

/*
 * Each tap divides the range between its reference and the next higher
 * tap: v = ref - (ref - upper) * (setting + base) / span. The VT tap is
 * selected from a fixed list of fractions of VREG instead.
 */
static const struct {
	u16 base;
	u16 span;
	bool ref_vt;	/* referenced to VT instead of VREG */
} s6e3aa2_taps[S6E3AA2_NUM_TAPS] = {
	{ 0, 860, false },					/* VT */
	{ 0, 256, false }, { 0, 256, false },			/* V1, V7 */
	{ 64, 320, true }, { 64, 320, true }, { 64, 320, true },	/* V11..V35 */
	{ 64, 320, true }, { 64, 320, true }, { 64, 320, true },	/* V51..V151 */
	{ 64, 320, true },					/* V203 */
	{ 129, 860, false },					/* V255 */
};

static const u8 s6e3aa2_vt_fraction[16] = {
	0, 12, 24, 36, 48, 60, 72, 84, 96, 108, 138, 148, 158, 168, 178, 186
};

/* Register setting of each tap that gives the nominal gamma curve */
static const u16 s6e3aa2_tap_default[S6E3AA2_NUM_TAPS] = {
	0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x100
};

/* ELVSS: 30 parameters, programmed per panel; we only change a few. */
#define ELVSS_LEN		30
#define ELVSS_MODE		0
#define ELVSS_MPS_CON		1
#define ELVSS_DIM_OFFSET	2
#define ELVSS_TSET		29
#define ELVSS_MODE_DEFAULT	0xa0
#define ELVSS_TSET_25C		0x19

/**
 * struct s6e3aa2_step - one calibrated luminance step
 * @grey: grey level each tap's voltage is taken from, compressing the
 *	  calibrated curve for this luminance
 * @offset: colour correction per tap (R, G, B)
 * @elvss_mps: ELVSS MPS_CON setting
 * @elvss_offset: ELVSS dimming offset
 */
struct s6e3aa2_step {
	u8 grey[S6E3AA2_NUM_TAPS];
	s8 offset[S6E3AA2_NUM_TAPS][3];
	u8 elvss_mps;
	u8 elvss_offset;
};

/**
 * struct s6e3aa2_level - one brightness level
 * @step: index into s6e3aa2_steps[]
 * @aor: lines of each frame kept dark (AMOLED off ratio)
 */
struct s6e3aa2_level {
	u8 step;
	u16 aor;
};

/* Luminance steps from 2 to 420 nit */
static const struct s6e3aa2_step s6e3aa2_steps[] = {
	{ /* 2 nit */
		.grey = { 0, 1, 47, 49, 52, 55, 59, 72, 99, 122, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -8, 0, -1 }, { -4, 0, -3 },
			{ -4, 1, -3 }, { -9, -2, -6 }, { -10, 1, -8 }, { -9, 0, -7 },
			{ -5, 0, -4 }, { -3, 1, -1 }, { -13, -7, -10 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x41,
	},
	{ /* 3 nit */
		.grey = { 0, 1, 42, 44, 46, 48, 53, 68, 96, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -8, 0, -2 }, { -7, -1, -7 },
			{ -9, -1, -5 }, { -6, 0, -5 }, { -9, 0, -10 }, { -9, -1, -7 },
			{ -4, -1, -3 }, { -2, 1, -1 }, { -6, -2, -4 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x42,
	},
	{ /* 4 nit */
		.grey = { 0, 1, 41, 42, 44, 46, 51, 67, 95, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, 0, -3 }, { -6, -1, -5 },
			{ -10, -1, -7 }, { -6, 1, -6 }, { -10, 0, -9 }, { -10, -1, -8 },
			{ -4, 0, -2 }, { -2, 0, -1 }, { -4, -1, -3 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x43,
	},
	{ /* 5 nit */
		.grey = { 0, 1, 40, 41, 43, 45, 50, 66, 94, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, 0, -3 }, { -8, -2, -7 },
			{ -10, -1, -6 }, { -10, -1, -7 }, { -13, -2, -12 }, { -10, -1, -8 },
			{ -3, -1, -2 }, { -2, 0, -1 }, { -5, -2, -4 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x44,
	},
	{ /* 6 nit */
		.grey = { 0, 1, 40, 41, 43, 45, 50, 66, 94, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, 0, -3 }, { -8, 0, -7 },
			{ -7, -1, -7 }, { -7, -1, -8 }, { -8, 1, -10 }, { -9, -1, -8 },
			{ -3, 0, -2 }, { -2, 0, -1 }, { -4, -1, -2 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x49,
	},
	{ /* 7 nit */
		.grey = { 0, 1, 39, 40, 42, 45, 50, 65, 94, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 0, -2 }, { -6, 0, -6 },
			{ -3, 2, -7 }, { -6, 1, -7 }, { -10, -1, -12 }, { -7, 0, -6 },
			{ -3, 1, -2 }, { -2, 0, 0 }, { -4, -1, -2 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x4d,
	},
	{ /* 8 nit */
		.grey = { 0, 1, 38, 39, 41, 44, 49, 65, 94, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 0, -3 }, { -4, -1, -7 },
			{ -2, 2, -7 }, { -7, 1, -7 }, { -7, 2, -9 }, { -8, 1, -5 },
			{ -2, 0, -2 }, { -2, 0, 0 }, { -4, 0, -1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x50,
	},
	{ /* 9 nit */
		.grey = { 0, 1, 37, 38, 40, 43, 48, 64, 93, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 0, -3 }, { -7, 0, -6 },
			{ -6, 2, -8 }, { -7, -1, -8 }, { -8, 1, -9 }, { -6, 1, -5 },
			{ -1, 1, -1 }, { -1, 0, 0 }, { -4, 0, -1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x51,
	},
	{ /* 10 nit */
		.grey = { 0, 1, 35, 36, 39, 42, 47, 63, 93, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -4, 0, -2 }, { -6, 1, -6 },
			{ -4, 1, -8 }, { -7, 0, -7 }, { -9, 0, -12 }, { -6, 2, -5 },
			{ 0, 0, -1 }, { -2, 0, -1 }, { -3, 0, 0 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x51,
	},
	{ /* 11 nit */
		.grey = { 0, 1, 34, 35, 38, 41, 47, 63, 93, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 0, -1 }, { -4, 0, -8 },
			{ -4, 1, -8 }, { -7, 2, -7 }, { -9, 0, -11 }, { -6, 1, -5 },
			{ 0, 0, -1 }, { -2, 0, -1 }, { -3, 0, 0 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x52,
	},
	{ /* 12 nit */
		.grey = { 0, 1, 33, 34, 37, 40, 46, 62, 92, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, -1, -2 }, { -6, 2, -6 },
			{ -8, -1, -10 }, { -5, 2, -7 }, { -10, 0, -10 }, { -6, 0, -4 },
			{ 0, 0, -1 }, { -2, 0, -1 }, { -3, 0, 0 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x52,
	},
	{ /* 13 nit */
		.grey = { 0, 1, 32, 33, 36, 39, 45, 61, 92, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, -1, -3 }, { -3, 2, -6 },
			{ -9, -1, -10 }, { -6, 1, -6 }, { -7, 0, -10 }, { -5, 0, -4 },
			{ 0, 1, 0 }, { -3, 0, -1 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x53,
	},
	{ /* 14 nit */
		.grey = { 0, 1, 32, 33, 36, 39, 45, 61, 92, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -2 }, { -4, 1, -8 },
			{ -7, 0, -9 }, { -9, 1, -8 }, { -9, -2, -10 }, { -6, 0, -5 },
			{ -1, 0, -1 }, { -2, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x53,
	},
	{ /* 15 nit */
		.grey = { 0, 1, 31, 32, 35, 38, 44, 61, 92, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -2 }, { -4, 2, -7 },
			{ -10, 0, -10 }, { -8, -1, -8 }, { -8, 0, -9 }, { -5, 0, -4 },
			{ -1, 0, -1 }, { -2, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 16 nit */
		.grey = { 0, 1, 29, 30, 33, 36, 43, 60, 92, 121, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -2 }, { -4, 2, -8 },
			{ -7, 0, -9 }, { -6, 2, -7 }, { -8, 0, -9 }, { -4, 2, -2 },
			{ 0, 0, -1 }, { -2, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 17 nit */
		.grey = { 0, 1, 28, 29, 32, 35, 42, 59, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -3 }, { -6, 2, -7 },
			{ -8, 0, -10 }, { -6, 1, -6 }, { -8, 0, -8 }, { -3, 2, -2 },
			{ 0, 1, 0 }, { -2, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 19 nit */
		.grey = { 0, 1, 27, 28, 31, 34, 41, 58, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -3 }, { -4, 1, -8 },
			{ -7, -1, -11 }, { -9, 1, -7 }, { -8, 0, -8 }, { -2, 2, -2 },
			{ -1, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 20 nit */
		.grey = { 0, 1, 27, 28, 31, 34, 41, 58, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -3 }, { -6, 0, -10 },
			{ -9, 0, -10 }, { -6, 0, -7 }, { -10, -2, -10 }, { -2, 1, -2 },
			{ 0, 1, 1 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 21 nit */
		.grey = { 0, 1, 26, 27, 30, 33, 40, 58, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, -1, -3 }, { -8, 0, -10 },
			{ -9, -1, -11 }, { -5, 1, -7 }, { -7, 1, -7 }, { -2, 1, -2 },
			{ -1, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 22 nit */
		.grey = { 0, 1, 25, 26, 29, 33, 40, 58, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, -1, -3 }, { -6, 0, -10 },
			{ -7, 2, -10 }, { -6, 0, -6 }, { -8, 1, -8 }, { -2, 0, -2 },
			{ -1, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 24 nit */
		.grey = { 0, 1, 24, 25, 28, 32, 39, 58, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, -1, -3 }, { -5, -1, -10 },
			{ -8, 1, -10 }, { -7, 1, -6 }, { -6, 1, -6 }, { -3, 0, -2 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 25 nit */
		.grey = { 0, 1, 24, 25, 28, 32, 39, 58, 91, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -2, -3 }, { -7, -1, -11 },
			{ -8, 1, -10 }, { -6, 0, -7 }, { -6, 1, -5 }, { -4, 0, -3 },
			{ 0, -1, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 27 nit */
		.grey = { 0, 1, 23, 24, 27, 31, 38, 57, 90, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -2, -3 }, { -8, -2, -12 },
			{ -9, 0, -10 }, { -7, 0, -7 }, { -5, 1, -5 }, { -4, 1, -1 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 29 nit */
		.grey = { 0, 1, 22, 23, 26, 30, 38, 57, 90, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, -2 }, { -9, -3, -13 },
			{ -9, 1, -10 }, { -5, 2, -5 }, { -6, 0, -5 }, { -3, 1, -1 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 30 nit */
		.grey = { 0, 1, 22, 23, 26, 30, 37, 56, 90, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -1, -1, -3 }, { -11, -3, -13 },
			{ -9, 0, -12 }, { -6, -1, -5 }, { -5, 1, -6 }, { -2, 2, 0 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 32 nit */
		.grey = { 0, 1, 20, 21, 24, 29, 37, 56, 90, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, -1, -3 }, { -7, -1, -11 },
			{ -6, 3, -8 }, { -3, 3, -3 }, { -5, 0, -6 }, { -3, 1, -1 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 34 nit */
		.grey = { 0, 1, 20, 21, 24, 29, 37, 56, 90, 120, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -1, -1, -3 }, { -8, -2, -12 },
			{ -7, 2, -9 }, { -3, 2, -3 }, { -6, 0, -6 }, { -3, 1, -1 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 37 nit */
		.grey = { 0, 1, 19, 20, 23, 28, 36, 56, 90, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, -4 }, { -9, -1, -12 },
			{ -7, 0, -9 }, { -3, 3, -3 }, { -5, 1, -5 }, { -3, 0, -1 },
			{ 0, 0, -1 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 39 nit */
		.grey = { 0, 1, 18, 20, 23, 28, 36, 56, 90, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 2, 3, -2 }, { -11, -1, -13 },
			{ -7, 0, -9 }, { -4, 2, -3 }, { -5, 0, -5 }, { -3, 0, -1 },
			{ 0, 0, -1 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x0c, .elvss_offset = 0x54,
	},
	{ /* 41 nit */
		.grey = { 0, 1, 17, 19, 23, 28, 36, 56, 90, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 3, 3, -1 }, { -3, 4, -10 },
			{ -11, 0, -9 }, { -3, 0, -4 }, { -5, 0, -5 }, { -2, 0, -1 },
			{ -1, 0, -1 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 44 nit */
		.grey = { 0, 1, 16, 18, 22, 27, 35, 55, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 1, 4, -1 }, { -5, 2, -10 },
			{ -8, 0, -9 }, { -3, 0, -3 }, { -4, 1, -5 }, { -1, 1, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 47 nit */
		.grey = { 0, 1, 15, 17, 21, 26, 35, 55, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 2, 3, -2 }, { -7, 3, -10 },
			{ -7, 1, -8 }, { 0, 3, 0 }, { -5, 0, -6 }, { -1, 1, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 50 nit */
		.grey = { 0, 1, 15, 17, 21, 26, 35, 55, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 1, 3, -2 }, { -8, 2, -12 },
			{ -7, -1, -9 }, { 0, 3, 0 }, { -6, 0, -6 }, { -1, 0, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 53 nit */
		.grey = { 0, 1, 14, 16, 20, 25, 34, 54, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 2, 3, -1 }, { -8, 0, -13 },
			{ -6, 1, -6 }, { -2, 2, -2 }, { -4, 1, -5 }, { 1, 1, 1 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 56 nit */
		.grey = { 0, 1, 14, 16, 20, 25, 34, 54, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 2, 2, -2 }, { -11, 0, -13 },
			{ -6, -1, -7 }, { 0, 2, -1 }, { -5, 1, -5 }, { 0, 0, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 60 nit */
		.grey = { 0, 1, 13, 15, 20, 25, 34, 54, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 2, 3, -1 }, { -5, 4, -10 },
			{ -7, -1, -7 }, { -2, 1, -2 }, { -5, 0, -5 }, { 0, 0, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 64 nit */
		.grey = { 0, 1, 12, 14, 19, 25, 33, 54, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 1, 2, -3 }, { -7, 3, -10 },
			{ -3, 3, -5 }, { -3, -2, -2 }, { -2, 1, -3 }, { 0, 0, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 68 nit */
		.grey = { 0, 1, 12, 14, 19, 25, 33, 54, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 1, 2, -3 }, { -8, 3, -10 },
			{ -3, 1, -5 }, { -2, -2, -2 }, { -4, 1, -4 }, { 0, -1, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 72 nit */
		.grey = { 0, 1, 11, 13, 18, 24, 33, 53, 89, 119, 146 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { 0, 3, -3 }, { -8, 3, -10 },
			{ -3, 2, -4 }, { -2, 0, -2 }, { -3, 0, -3 }, { 0, 0, 1 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 77 nit */
		.grey = { 0, 1, 11, 13, 18, 24, 33, 53, 90, 121, 147 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 2, -5 }, { -12, 0, -13 },
			{ -3, 2, -4 }, { -3, 0, -1 }, { -3, 0, -5 }, { -1, 0, 0 },
			{ 0, 0, 0 }, { 0, 0, -1 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 82 nit */
		.grey = { 0, 1, 11, 13, 18, 25, 34, 55, 92, 123, 151 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 2, -5 }, { -12, 0, -13 },
			{ -3, 3, -3 }, { -3, 0, -1 }, { -3, 0, -5 }, { 0, -1, 0 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -2, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x54,
	},
	{ /* 87 nit */
		.grey = { 0, 1, 11, 13, 18, 24, 34, 56, 94, 125, 154 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 1, -6 }, { -14, -1, -13 },
			{ -4, 1, -3 }, { -2, 1, -1 }, { -2, 1, -3 }, { -1, 0, 0 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x53,
	},
	{ /* 93 nit */
		.grey = { 0, 1, 11, 13, 19, 25, 35, 57, 97, 129, 159 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 1, -6 }, { -12, 1, -12 },
			{ -6, -1, -4 }, { -2, 0, -1 }, { -3, 0, -4 }, { 0, 0, 0 },
			{ 0, 0, 1 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x53,
	},
	{ /* 98 nit */
		.grey = { 0, 1, 11, 13, 19, 26, 36, 59, 99, 131, 162 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 1, -7 }, { -12, 1, -11 },
			{ -4, 0, -2 }, { -2, 0, -1 }, { -3, 0, -4 }, { 0, -1, -1 },
			{ 0, 0, 0 }, { 0, 0, 1 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x53,
	},
	{ /* 105 nit */
		.grey = { 0, 1, 11, 13, 19, 26, 36, 60, 102, 135, 167 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -6, 1, -8 }, { -13, -2, -12 },
			{ -4, 0, -2 }, { -2, 0, -1 }, { -1, 0, -3 }, { 0, 0, 1 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x52,
	},
	{ /* 111 nit */
		.grey = { 0, 1, 11, 13, 19, 27, 37, 62, 104, 139, 171 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, 0, -9 }, { -12, -1, -11 },
			{ -3, 1, -2 }, { -2, 0, -1 }, { -2, 0, -3 }, { 0, 0, 1 },
			{ 0, 0, 1 }, { 0, 0, -1 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x52,
	},
	{ /* 119 nit */
		.grey = { 0, 1, 11, 13, 20, 28, 39, 64, 108, 143, 177 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -5, 1, -7 }, { -12, 1, -11 },
			{ -5, 0, -3 }, { -3, 0, -1 }, { -1, 0, -3 }, { 1, 0, 1 },
			{ 0, 0, 0 }, { -1, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x52,
	},
	{ /* 126 nit */
		.grey = { 0, 1, 11, 13, 20, 28, 39, 65, 111, 146, 181 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -5, 0, -8 }, { -12, 1, -11 },
			{ -4, 0, -2 }, { -2, 0, -1 }, { -1, 0, -2 }, { -1, 0, 0 },
			{ 0, 0, 1 }, { -1, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x52,
	},
	{ /* 134 nit */
		.grey = { 0, 1, 11, 13, 21, 29, 40, 67, 114, 149, 185 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, 0, -9 }, { -12, 1, -11 },
			{ -4, -3, -3 }, { -2, 0, -1 }, { -1, 0, -2 }, { -1, 0, 0 },
			{ 0, 0, 1 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 143 nit */
		.grey = { 0, 1, 11, 13, 21, 30, 41, 69, 117, 154, 191 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, -1, -9 }, { -11, 0, -9 },
			{ -3, 0, -3 }, { -2, 0, -1 }, { -2, 0, -2 }, { 1, 0, 1 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 152 nit */
		.grey = { 0, 1, 11, 13, 21, 30, 42, 70, 120, 158, 195 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -8, -1, -9 }, { -9, 0, -8 },
			{ -3, 0, -3 }, { -2, 0, -1 }, { 0, 0, -1 }, { 1, 0, 1 },
			{ -1, 0, 0 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 162 nit */
		.grey = { 0, 1, 11, 14, 21, 31, 43, 72, 123, 162, 201 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -7, 2, -9 }, { -12, -2, -10 },
			{ -3, 0, -3 }, { -2, 0, -1 }, { -2, 0, -2 }, { 1, 0, 1 },
			{ -1, 0, 0 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 172 nit */
		.grey = { 0, 1, 11, 14, 22, 32, 44, 74, 126, 166, 206 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -8, 1, -10 }, { -11, -2, -8 },
			{ -3, 0, -3 }, { -2, 0, -1 }, { -1, 0, -1 }, { 1, 0, 1 },
			{ -1, 0, -1 }, { 0, 0, 0 }, { 0, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 183 nit */
		.grey = { 0, 1, 11, 14, 22, 32, 45, 76, 129, 171, 212 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -8, 1, -9 }, { -9, -1, -7 },
			{ -3, 0, -3 }, { -2, 0, -1 }, { 0, 0, 0 }, { 1, 0, 1 },
			{ 0, 0, -1 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 195 nit */
		.grey = { 0, 1, 10, 13, 22, 32, 45, 76, 129, 171, 212 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -3, 3, -6 }, { -8, 0, -6 },
			{ -3, 0, -2 }, { -2, 0, -1 }, { 0, 0, -1 }, { 1, 0, 2 },
			{ 0, 0, -1 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 207 nit */
		.grey = { 0, 1, 10, 13, 22, 32, 45, 76, 129, 171, 212 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -5, 1, -7 }, { -8, 0, -5 },
			{ -3, 0, -3 }, { -1, 0, 0 }, { 0, 0, -1 }, { 1, 0, 2 },
			{ 0, 0, -1 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 220 nit */
		.grey = { 0, 1, 9, 12, 22, 31, 45, 76, 129, 170, 212 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 2, -5 }, { -8, 1, -6 },
			{ -2, 0, -2 }, { 0, 0, 0 }, { 0, 0, -1 }, { 1, 0, 2 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 234 nit */
		.grey = { 0, 1, 9, 12, 21, 31, 45, 76, 129, 170, 212 },
		.offset = {
			{ 0, 0, 0 }, { 0, 0, 0 }, { -2, 2, -4 }, { -8, 0, -7 },
			{ -2, 0, -1 }, { 0, 0, 0 }, { 0, 0, 0 }, { 1, 0, 1 },
			{ 0, 0, 0 }, { 0, 0, 0 }, { -1, 0, 1 },
		},
		.elvss_mps = 0x1c, .elvss_offset = 0x51,
	},
	{ /* 249 nit */
		.grey = { 0, 1, 8, 11, 21, 31, 45, 76, 129, 171, 213 },
		.elvss_mps = 0x1c, .elvss_offset = 0x50,
	},
	{ /* 265 nit */
		.grey = { 0, 1, 8, 11, 22, 32, 46, 78, 133, 175, 218 },
		.elvss_mps = 0x1c, .elvss_offset = 0x50,
	},
	{ /* 282 nit */
		.grey = { 0, 1, 8, 11, 22, 32, 46, 79, 135, 179, 224 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4f,
	},
	{ /* 300 nit */
		.grey = { 0, 1, 8, 12, 22, 33, 48, 81, 139, 184, 229 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4e,
	},
	{ /* 316 nit */
		.grey = { 0, 1, 8, 12, 22, 34, 49, 83, 142, 189, 235 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4e,
	},
	{ /* 333 nit */
		.grey = { 0, 1, 8, 12, 23, 34, 50, 84, 144, 192, 240 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4d,
	},
	{ /* 350 nit */
		.grey = { 0, 1, 8, 12, 23, 35, 50, 86, 147, 196, 245 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4d,
	},
	{ /* 357 nit */
		.grey = { 0, 1, 8, 11, 23, 35, 51, 87, 148, 199, 248 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4c,
	},
	{ /* 365 nit */
		.grey = { 0, 1, 8, 12, 23, 35, 51, 87, 149, 200, 250 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4c,
	},
	{ /* 372 nit */
		.grey = { 0, 1, 7, 11, 23, 35, 51, 87, 149, 200, 250 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4c,
	},
	{ /* 380 nit */
		.grey = { 0, 1, 7, 11, 23, 35, 51, 86, 149, 200, 250 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4b,
	},
	{ /* 387 nit */
		.grey = { 0, 1, 7, 11, 23, 35, 51, 86, 149, 200, 250 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4b,
	},
	{ /* 395 nit */
		.grey = { 0, 1, 7, 11, 23, 35, 51, 86, 149, 200, 250 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4b,
	},
	{ /* 403 nit */
		.grey = { 0, 1, 7, 12, 24, 36, 52, 88, 150, 201, 251 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4b,
	},
	{ /* 412 nit */
		.grey = { 0, 1, 8, 12, 24, 36, 52, 88, 151, 202, 253 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4a,
	},
	{ /* 420 nit */
		.grey = { 0, 1, 7, 11, 23, 35, 51, 87, 151, 203, 255 },
		.elvss_mps = 0x1c, .elvss_offset = 0x4a,
	},
};

/* Luminance step and AOR period (lines) for each brightness level */
static const struct s6e3aa2_level s6e3aa2_levels[S6E3AA2_NUM_LEVELS] = {
	{  0, 1293 }, {  1, 1292 }, {  1, 1286 }, {  1, 1283 },	/* 0 */
	{  1, 1281 }, {  2, 1279 }, {  2, 1277 }, {  2, 1270 },	/* 4 */
	{  2, 1268 }, {  3, 1267 }, {  3, 1265 }, {  3, 1264 },	/* 8 */
	{  3, 1263 }, {  4, 1261 }, {  4, 1251 }, {  4, 1249 },	/* 12 */
	{  4, 1248 }, {  5, 1247 }, {  5, 1245 }, {  5, 1238 },	/* 16 */
	{  5, 1235 }, {  6, 1233 }, {  6, 1231 }, {  6, 1229 },	/* 20 */
	{  6, 1228 }, {  7, 1219 }, {  7, 1218 }, {  7, 1217 },	/* 24 */
	{  7, 1215 }, {  8, 1213 }, {  8, 1208 }, {  8, 1203 },	/* 28 */
	{  8, 1202 }, {  9, 1199 }, {  9, 1197 }, {  9, 1190 },	/* 32 */
	{  9, 1188 }, { 10, 1185 }, { 10, 1184 }, { 10, 1183 },	/* 36 */
	{ 10, 1182 }, { 11, 1171 }, { 11, 1170 }, { 11, 1169 },	/* 40 */
	{ 12, 1165 }, { 12, 1156 }, { 12, 1155 }, { 13, 1150 },	/* 44 */
	{ 13, 1145 }, { 14, 1140 }, { 14, 1136 }, { 15, 1130 },	/* 48 */
	{ 15, 1123 }, { 16, 1113 }, { 16, 1103 }, { 17, 1097 },	/* 52 */
	{ 17, 1090 }, { 18, 1083 }, { 18, 1076 }, { 19, 1073 },	/* 56 */
	{ 19, 1070 }, { 20, 1057 }, { 20, 1044 }, { 21, 1041 },	/* 60 */
	{ 21, 1038 }, { 22, 1025 }, { 22, 1011 }, { 23, 1001 },	/* 64 */
	{ 23,  992 }, { 24,  985 }, { 24,  978 }, { 25,  968 },	/* 68 */
	{ 25,  959 }, { 26,  946 }, { 26,  932 }, { 27,  916 },	/* 72 */
	{ 27,  899 }, { 28,  889 }, { 28,  880 }, { 29,  866 },	/* 76 */
	{ 29,  852 }, { 30,  836 }, { 30,  820 }, { 31,  803 },	/* 80 */
	{ 31,  786 }, { 32,  770 }, { 32,  754 }, { 33,  737 },	/* 84 */
	{ 33,  720 }, { 34,  703 }, { 34,  686 }, { 35,  663 },	/* 88 */
	{ 35,  640 }, { 36,  616 }, { 36,  592 }, { 37,  569 },	/* 92 */
	{ 37,  545 }, { 38,  521 }, { 38,  496 }, { 39,  478 },	/* 96 */
	{ 39,  450 }, { 40,  476 }, { 40,  450 }, { 41,  475 },	/* 100 */
	{ 41,  450 }, { 42,  478 }, { 42,  450 }, { 43,  472 },	/* 104 */
	{ 43,  450 }, { 44,  478 }, { 44,  450 }, { 45,  473 },	/* 108 */
	{ 45,  450 }, { 46,  479 }, { 46,  450 }, { 47,  474 },	/* 112 */
	{ 47,  450 }, { 48,  475 }, { 48,  450 }, { 49,  477 },	/* 116 */
	{ 49,  450 }, { 50,  475 }, { 50,  450 }, { 51,  476 },	/* 120 */
	{ 51,  450 }, { 52,  475 }, { 52,  450 }, { 53,  476 },	/* 124 */
	{ 53,  450 }, { 54,  441 }, { 54,  432 }, { 54,  423 },	/* 128 */
	{ 54,  414 }, { 54,  405 }, { 54,  396 }, { 54,  387 },	/* 132 */
	{ 55,  378 }, { 55,  370 }, { 55,  361 }, { 55,  353 },	/* 136 */
	{ 55,  344 }, { 55,  336 }, { 55,  327 }, { 56,  317 },	/* 140 */
	{ 56,  308 }, { 56,  298 }, { 56,  288 }, { 56,  278 },	/* 144 */
	{ 56,  269 }, { 56,  259 }, { 57,  250 }, { 57,  242 },	/* 148 */
	{ 57,  233 }, { 57,  224 }, { 57,  215 }, { 57,  207 },	/* 152 */
	{ 57,  198 }, { 57,  189 }, { 58,  190 }, { 58,  181 },	/* 156 */
	{ 58,  172 }, { 58,  163 }, { 58,  155 }, { 58,  146 },	/* 160 */
	{ 58,  137 }, { 58,  128 }, { 59,  191 }, { 59,  183 },	/* 164 */
	{ 59,  175 }, { 59,  167 }, { 59,  160 }, { 59,  152 },	/* 168 */
	{ 59,  144 }, { 59,  136 }, { 59,  128 }, { 60,  191 },	/* 172 */
	{ 60,  183 }, { 60,  175 }, { 60,  167 }, { 60,  160 },	/* 176 */
	{ 60,  152 }, { 60,  144 }, { 60,  136 }, { 60,  128 },	/* 180 */
	{ 61,  192 }, { 61,  184 }, { 61,  177 }, { 61,  170 },	/* 184 */
	{ 61,  163 }, { 61,  156 }, { 61,  149 }, { 61,  142 },	/* 188 */
	{ 61,  135 }, { 61,  128 }, { 62,  180 }, { 62,  173 },	/* 192 */
	{ 62,  165 }, { 62,  158 }, { 62,  150 }, { 62,  143 },	/* 196 */
	{ 62,  135 }, { 62,  128 }, { 63,  181 }, { 63,  175 },	/* 200 */
	{ 63,  168 }, { 63,  161 }, { 63,  155 }, { 63,  148 },	/* 204 */
	{ 63,  141 }, { 63,  135 }, { 63,  128 }, { 64,  179 },	/* 208 */
	{ 64,  172 }, { 64,  166 }, { 64,  160 }, { 64,  153 },	/* 212 */
	{ 64,  147 }, { 64,  141 }, { 64,  134 }, { 64,  128 },	/* 216 */
	{ 65,  145 }, { 65,  140 }, { 65,  134 }, { 65,  128 },	/* 220 */
	{ 66,  147 }, { 66,  141 }, { 66,  134 }, { 66,  128 },	/* 224 */
	{ 67,  122 }, { 67,  115 }, { 67,  109 }, { 68,  102 },	/* 228 */
	{ 68,   95 }, { 68,   88 }, { 68,   81 }, { 69,   76 },	/* 232 */
	{ 69,   71 }, { 69,   66 }, { 69,   61 }, { 70,   52 },	/* 236 */
	{ 70,   44 }, { 70,   35 }, { 70,   26 }, { 71,   31 },	/* 240 */
	{ 71,   25 }, { 71,   18 }, { 71,   12 }, { 72,   33 },	/* 244 */
	{ 72,   26 }, { 72,   19 }, { 72,   12 }, { 73,   30 },	/* 248 */
	{ 73,   24 }, { 73,   18 }, { 73,   12 }, { 73,   12 },	/* 252 */
};

#define S6E3AA2_NUM_STEPS	ARRAY_SIZE(s6e3aa2_steps)

struct s6e3aa2 {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct backlight_device *bl;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data *supplies;
	u8 elvss[1 + ELVSS_LEN];
	u8 (*gamma)[1 + S6E3AA2_GAMMA_LEN];	/* per step, with command */
	bool calibrated;
};

/* Enabled in this order, disabled in reverse. */
static const struct regulator_bulk_data s6e3aa2_supplies[] = {
	{ .supply = "vci" },
	{ .supply = "vdd" },
};

static inline struct s6e3aa2 *to_s6e3aa2(struct drm_panel *panel)
{
	return container_of(panel, struct s6e3aa2, panel);
}

/*
 * Unpack the MTP (or gamma register) layout into per-tap values: V255 as
 * sign bit and magnitude in two bytes per colour, V203 to V1 as one byte
 * per colour with the sign in bit 7, VT as unsigned nibbles R, G and B.
 */
static void s6e3aa2_decode_mtp(const u8 *raw, int mtp[][3])
{
	int tap, c, i = 0;

	for (c = 0; c < 3; c++, i += 2)
		mtp[S6E3AA2_TAP_TOP][c] = raw[i] & 0x01 ? -raw[i + 1] : raw[i + 1];

	for (tap = S6E3AA2_TAP_TOP - 1; tap > S6E3AA2_TAP_VT; tap--)
		for (c = 0; c < 3; c++, i++)
			mtp[tap][c] = raw[i] & 0x80 ? -(raw[i] & 0x7f) : raw[i];

	mtp[S6E3AA2_TAP_VT][0] = raw[i] >> 4;
	mtp[S6E3AA2_TAP_VT][1] = raw[i] & 0x0f;
	mtp[S6E3AA2_TAP_VT][2] = raw[i + 1] & 0x0f;
}

static void s6e3aa2_encode_gamma(const int gamma[][3], u8 *raw)
{
	int tap, c, i = 0;

	for (c = 0; c < 3; c++) {
		raw[i++] = gamma[S6E3AA2_TAP_TOP][c] >> 8;
		raw[i++] = gamma[S6E3AA2_TAP_TOP][c] & 0xff;
	}

	for (tap = S6E3AA2_TAP_TOP - 1; tap > S6E3AA2_TAP_VT; tap--)
		for (c = 0; c < 3; c++)
			raw[i++] = gamma[tap][c];

	raw[i++] = gamma[S6E3AA2_TAP_VT][0] << 4 | gamma[S6E3AA2_TAP_VT][1];
	raw[i] = gamma[S6E3AA2_TAP_VT][2];
}

/* Voltage of a tap's reference: VREG, or the VT voltage */
static int s6e3aa2_tap_ref(int tap, const int volt[][3], int c)
{
	return s6e3aa2_taps[tap].ref_vt ? volt[S6E3AA2_TAP_VT][c] : S6E3AA2_VREG;
}

/* Output voltages of the taps with the calibrated (MTP) settings */
static void s6e3aa2_tap_voltages(const int mtp[][3], int volt[][3])
{
	int tap, c;

	for (c = 0; c < 3; c++) {
		int sel = s6e3aa2_tap_default[S6E3AA2_TAP_VT] +
			  mtp[S6E3AA2_TAP_VT][c];

		volt[S6E3AA2_TAP_VT][c] = S6E3AA2_VREG -
			S6E3AA2_VREG * s6e3aa2_vt_fraction[sel] /
			s6e3aa2_taps[S6E3AA2_TAP_VT].span;
	}

	for (tap = S6E3AA2_TAP_TOP; tap > S6E3AA2_TAP_VT; tap--) {
		for (c = 0; c < 3; c++) {
			int ref = s6e3aa2_tap_ref(tap, volt, c);
			int upper = tap == S6E3AA2_TAP_TOP ? 0 : volt[tap + 1][c];
			int sel = s6e3aa2_tap_default[tap] + mtp[tap][c] +
				  s6e3aa2_taps[tap].base;

			volt[tap][c] = ref - (ref - upper) * sel /
				       s6e3aa2_taps[tap].span;
		}
	}
}

/* Calibrated voltage of any grey level, interpolated between the taps */
static int s6e3aa2_grey_voltage(const int volt[][3], int grey, int c)
{
	int tap = 1;

	while (s6e3aa2_tap_grey[tap] < grey)
		tap++;

	return volt[tap][c] + (volt[tap - 1][c] - volt[tap][c]) *
	       (s6e3aa2_tap_grey[tap] - grey) /
	       (s6e3aa2_tap_grey[tap] - s6e3aa2_tap_grey[tap - 1]);
}

/*
 * Gamma settings for one luminance step: pick the calibrated voltage of
 * the step's grey level for every tap and turn it back into a register
 * setting, relative to the factory calibration.
 */
static void s6e3aa2_step_gamma(const struct s6e3aa2_step *step,
			       const int mtp[][3], const int volt[][3],
			       int gamma[][3])
{
	int target[S6E3AA2_NUM_TAPS][3];
	int tap, c;

	for (c = 0; c < 3; c++) {
		target[S6E3AA2_TAP_VT][c] = volt[S6E3AA2_TAP_VT][c];
		for (tap = S6E3AA2_TAP_VT + 1; tap < S6E3AA2_NUM_TAPS; tap++)
			target[tap][c] = s6e3aa2_grey_voltage(volt,
							      step->grey[tap], c);
		gamma[S6E3AA2_TAP_VT][c] = s6e3aa2_tap_default[S6E3AA2_TAP_VT];
	}

	for (tap = S6E3AA2_TAP_TOP; tap > S6E3AA2_TAP_VT; tap--) {
		for (c = 0; c < 3; c++) {
			int ref = s6e3aa2_tap_ref(tap, target, c);
			int upper = tap == S6E3AA2_TAP_TOP ? 0 : target[tap + 1][c];
			int val;

			val = (ref - target[tap][c] + 1) * s6e3aa2_taps[tap].span /
			      (ref - upper) - s6e3aa2_taps[tap].base;
			val += step->offset[tap][c] - mtp[tap][c];

			val = max(val, 0);
			if (tap != S6E3AA2_TAP_TOP)
				val = min(val, 255);
			gamma[tap][c] = val;
		}
	}
}

static void s6e3aa2_calc_gamma(struct s6e3aa2 *ctx, const u8 *mtp_raw)
{
	int mtp[S6E3AA2_NUM_TAPS][3], volt[S6E3AA2_NUM_TAPS][3];
	int gamma[S6E3AA2_NUM_TAPS][3];
	int i;

	s6e3aa2_decode_mtp(mtp_raw, mtp);
	s6e3aa2_tap_voltages(mtp, volt);

	for (i = 0; i < S6E3AA2_NUM_STEPS; i++) {
		s6e3aa2_step_gamma(&s6e3aa2_steps[i], mtp, volt, gamma);
		ctx->gamma[i][0] = MCS_GAMMACTL;
		s6e3aa2_encode_gamma(gamma, &ctx->gamma[i][1]);
	}
}

static void s6e3aa2_set_brightness(struct s6e3aa2 *ctx,
				   struct mipi_dsi_multi_context *dsi,
				   unsigned int brightness)
{
	const struct s6e3aa2_level *level = &s6e3aa2_levels[brightness];
	const struct s6e3aa2_step *step = &s6e3aa2_steps[level->step];

	mipi_dsi_dcs_write_buffer_multi(dsi, ctx->gamma[level->step],
					sizeof(ctx->gamma[0]));
	/* AOR[10:8] go into the top bits of the 10th parameter. */
	mipi_dsi_dcs_write_var_seq_multi(dsi, MCS_AIDCTL,
					 0xff, 0x20, 0x1a, 0x33, 0x5e, 0x8c, 0xb3,
					 0xd9, 0xff, (level->aor >> 8) << 5,
					 level->aor & 0xff);

	ctx->elvss[1 + ELVSS_MODE] = ELVSS_MODE_DEFAULT;
	ctx->elvss[1 + ELVSS_MPS_CON] = step->elvss_mps;
	ctx->elvss[1 + ELVSS_DIM_OFFSET] = step->elvss_offset;
	/* No panel temperature sensor is wired up: assume 25 degC. */
	ctx->elvss[1 + ELVSS_TSET] = ELVSS_TSET_25C;
	mipi_dsi_dcs_write_buffer_multi(dsi, ctx->elvss, sizeof(ctx->elvss));

	mipi_dsi_dcs_write_seq_multi(dsi, MCS_GAMMAUPD, 0x03);
}

static int s6e3aa2_update_status(struct backlight_device *bl)
{
	struct s6e3aa2 *ctx = bl_get_data(bl);
	struct mipi_dsi_multi_context dsi = { .dsi = ctx->dsi };

	if (!ctx->panel.prepared)
		return 0;

	samsung_dsi_test_key_on_lvl2(&dsi);
	samsung_dsi_test_key_on_lvl3(&dsi);
	s6e3aa2_set_brightness(ctx, &dsi, backlight_get_brightness(bl));
	samsung_dsi_test_key_off_lvl3(&dsi);
	samsung_dsi_test_key_off_lvl2(&dsi);

	return dsi.accum_err;
}

static int s6e3aa2_read(struct s6e3aa2 *ctx, u8 cmd, void *data, size_t len)
{
	int ret;

	ret = mipi_dsi_set_maximum_return_packet_size(ctx->dsi, len);
	if (ret)
		return ret;

	ret = mipi_dsi_dcs_read(ctx->dsi, cmd, data, len);
	if (ret < 0)
		return ret;

	return ret == len ? 0 : -EIO;
}

/*
 * Read what the panel was calibrated with at the factory: the ID, the
 * gamma calibration (MTP) and the ELVSS settings. The test keys must be
 * open.
 */
static int s6e3aa2_read_calibration(struct s6e3aa2 *ctx)
{
	struct device *dev = &ctx->dsi->dev;
	u8 id[3], mtp[S6E3AA2_GAMMA_LEN];
	int ret;

	ret = s6e3aa2_read(ctx, MIPI_DCS_GET_DISPLAY_ID, id, sizeof(id));
	if (ret)
		return ret;

	/* The controller answers with an all-zero ID without a working panel. */
	if (!memchr_inv(id, 0, sizeof(id))) {
		dev_err(dev, "no panel connected (ID 000000)\n");
		return -ENODEV;
	}

	ret = s6e3aa2_read(ctx, MCS_MTP, mtp, sizeof(mtp));
	if (ret)
		return ret;

	ret = s6e3aa2_read(ctx, MCS_ELVSS, &ctx->elvss[1], ELVSS_LEN);
	if (ret)
		return ret;
	ctx->elvss[0] = MCS_ELVSS;

	s6e3aa2_calc_gamma(ctx, mtp);
	ctx->calibrated = true;

	dev_info(dev, "panel ID %*phN\n", (int)sizeof(id), id);
	dev_dbg(dev, "MTP %*ph\n", (int)sizeof(mtp), mtp);

	return 0;
}

static void s6e3aa2_reset(struct s6e3aa2 *ctx)
{
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ctx->reset_gpio, 0);
	usleep_range(10000, 11000);
}

static int s6e3aa2_on(struct s6e3aa2 *ctx)
{
	struct mipi_dsi_multi_context dsi = { .dsi = ctx->dsi };
	int ret;

	mipi_dsi_dcs_exit_sleep_mode_multi(&dsi);
	mipi_dsi_msleep(&dsi, 20);

	samsung_dsi_test_key_on_lvl2(&dsi);
	samsung_dsi_test_key_on_lvl3(&dsi);
	if (dsi.accum_err)
		return dsi.accum_err;

	if (!ctx->calibrated) {
		ret = s6e3aa2_read_calibration(ctx);
		if (ret) {
			dev_err(&ctx->dsi->dev,
				"failed to read panel calibration: %d\n", ret);
			return ret;
		}
	}

	/* Crack detection active low; error flag on the ERR_FG pin */
	mipi_dsi_dcs_write_seq_multi(&dsi, MCS_PCD, 0x5c);
	mipi_dsi_dcs_write_seq_multi(&dsi, MCS_ERRFLAG, 0x44);
	s6e3aa2_set_brightness(ctx, &dsi, ctx->bl->props.brightness);
	samsung_dsi_test_key_off_lvl3(&dsi);
	samsung_dsi_test_key_off_lvl2(&dsi);

	mipi_dsi_dcs_set_tear_on_multi(&dsi, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	mipi_dsi_msleep(&dsi, 120);

	return dsi.accum_err;
}

static int s6e3aa2_prepare(struct drm_panel *panel)
{
	struct s6e3aa2 *ctx = to_s6e3aa2(panel);
	struct device *dev = &ctx->dsi->dev;
	int i, ret;

	/* The supplies have to come up in order, so no bulk enable. */
	for (i = 0; i < ARRAY_SIZE(s6e3aa2_supplies); i++) {
		ret = regulator_enable(ctx->supplies[i].consumer);
		if (ret) {
			dev_err(dev, "failed to enable %s supply: %d\n",
				ctx->supplies[i].supply, ret);
			goto err_disable;
		}
	}
	usleep_range(10000, 11000);

	s6e3aa2_reset(ctx);

	ret = s6e3aa2_on(ctx);
	if (ret) {
		gpiod_set_value_cansleep(ctx->reset_gpio, 1);
		goto err_disable;
	}

	return 0;

err_disable:
	while (--i >= 0)
		regulator_disable(ctx->supplies[i].consumer);

	return ret;
}

static int s6e3aa2_unprepare(struct drm_panel *panel)
{
	struct s6e3aa2 *ctx = to_s6e3aa2(panel);
	struct mipi_dsi_multi_context dsi = { .dsi = ctx->dsi };
	int i;

	mipi_dsi_dcs_enter_sleep_mode_multi(&dsi);
	mipi_dsi_msleep(&dsi, 150);

	gpiod_set_value_cansleep(ctx->reset_gpio, 1);

	for (i = ARRAY_SIZE(s6e3aa2_supplies) - 1; i >= 0; i--)
		regulator_disable(ctx->supplies[i].consumer);

	return 0;
}

static int s6e3aa2_enable(struct drm_panel *panel)
{
	struct s6e3aa2 *ctx = to_s6e3aa2(panel);
	struct mipi_dsi_multi_context dsi = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_on_multi(&dsi);

	return dsi.accum_err;
}

static int s6e3aa2_disable(struct drm_panel *panel)
{
	struct s6e3aa2 *ctx = to_s6e3aa2(panel);
	struct mipi_dsi_multi_context dsi = { .dsi = ctx->dsi };

	mipi_dsi_dcs_set_display_off_multi(&dsi);
	mipi_dsi_usleep_range(&dsi, 10000, 11000);

	return dsi.accum_err;
}

/*
 * In command mode the porches only shape the transfer of a frame; the
 * panel refreshes at 60 Hz by itself.
 */
static const struct drm_display_mode s6e3aa2_mode = {
	.clock = (720 + 1 + 1 + 1) * (1280 + 3 + 1 + 10) * 60 / 1000,
	.hdisplay = 720,
	.hsync_start = 720 + 1,
	.hsync_end = 720 + 1 + 1,
	.htotal = 720 + 1 + 1 + 1,
	.vdisplay = 1280,
	.vsync_start = 1280 + 3,
	.vsync_end = 1280 + 3 + 1,
	.vtotal = 1280 + 3 + 1 + 10,
	.width_mm = 59,
	.height_mm = 105,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int s6e3aa2_get_modes(struct drm_panel *panel,
			     struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &s6e3aa2_mode);
}

static const struct backlight_ops s6e3aa2_bl_ops = {
	.update_status = s6e3aa2_update_status,
};

static const struct drm_panel_funcs s6e3aa2_panel_funcs = {
	.prepare = s6e3aa2_prepare,
	.unprepare = s6e3aa2_unprepare,
	.enable = s6e3aa2_enable,
	.disable = s6e3aa2_disable,
	.get_modes = s6e3aa2_get_modes,
};

static int s6e3aa2_probe(struct mipi_dsi_device *dsi)
{
	const struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
		.brightness = S6E3AA2_DEFAULT_LEVEL,
		.max_brightness = S6E3AA2_NUM_LEVELS - 1,
	};
	struct device *dev = &dsi->dev;
	struct s6e3aa2 *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct s6e3aa2, panel,
				   &s6e3aa2_panel_funcs,
				   DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);

	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->gamma = devm_kzalloc_objs(dev, *ctx->gamma, S6E3AA2_NUM_STEPS);
	if (!ctx->gamma)
		return -ENOMEM;

	ret = devm_regulator_bulk_get_const(dev, ARRAY_SIZE(s6e3aa2_supplies),
					    s6e3aa2_supplies, &ctx->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "failed to get regulators\n");

	/* Leave the panel as the bootloader set it up until prepare. */
	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ctx->reset_gpio),
				     "failed to get reset GPIO\n");

	ctx->bl = devm_backlight_device_register(dev, dev_name(dev), dev, ctx,
						 &s6e3aa2_bl_ops, &props);
	if (IS_ERR(ctx->bl))
		return dev_err_probe(dev, PTR_ERR(ctx->bl),
				     "failed to register backlight device\n");

	/* drm_panel_enable() and drm_panel_disable() switch it on and off. */
	ctx->panel.backlight = ctx->bl;

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	ctx->panel.prepare_prev_first = true;
	drm_panel_add(&ctx->panel);

	ret = devm_mipi_dsi_attach(dev, dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	return 0;
}

static void s6e3aa2_remove(struct mipi_dsi_device *dsi)
{
	struct s6e3aa2 *ctx = mipi_dsi_get_drvdata(dsi);

	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id s6e3aa2_of_match[] = {
	{ .compatible = "samsung,s6e3aa2-ams474mm01" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s6e3aa2_of_match);

static struct mipi_dsi_driver s6e3aa2_driver = {
	.probe = s6e3aa2_probe,
	.remove = s6e3aa2_remove,
	.driver = {
		.name = "panel-samsung-s6e3aa2",
		.of_match_table = s6e3aa2_of_match,
	},
};
module_mipi_dsi_driver(s6e3aa2_driver);

MODULE_AUTHOR("Martin Štrobl <recordyletsplays@gmail.com>");
MODULE_DESCRIPTION("Samsung AMS474MM01 panel with S6E3AA2 controller");
MODULE_LICENSE("GPL");
