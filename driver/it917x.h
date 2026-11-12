// SPDX-License-Identifier: GPL-2.0-only
/*
 * ITE IT917x driver definitions (it917x.h)
 *
 */

#ifndef __IT917X_H__
#define __IT917X_H__

#ifdef __linux__
#include <linux/types.h>
#include <linux/device.h>
#elif defined(_WIN32) || defined(_WIN64)
#include "misc_win.h"
#endif

#include "i2c_comm.h"

#define IT917X_CMD_REG_READ			0x00
#define IT917X_CMD_REG_WRITE		0x01
#define IT917X_CMD_QUERYINFO		0x22
#define IT917X_CMD_BOOT				0x23
#define IT917X_CMD_FW_SCATTER_WRITE	0x29

/*
 * IT917x/IT9173はチップ内部に LINK / OFDM の2つのプロセッサを持ち、
 * どちらに対するREG_READ/REG_WRITEかによってcmdの上位ビットが変わる。
 *
 * LINK: REG_READ=0x0000, REG_WRITE=0x0001
 * OFDM: REG_READ=0x8000, REG_WRITE=0x8001
 *
 */
enum it917x_proc {
	IT917X_PROC_LINK = 0,
	IT917X_PROC_OFDM = 0x8000,
};

struct it917x_config {
	u16 chip_id;
	u8 chip_version;
	u8 tuner_id;
	u8 i2c_speed;
	u32 xtal_hz;
	u32 adc_hz;
	u32 if_freq;
	bool inversion;
	bool clock_out;
};

static const struct it917x_config it917x_default_config = {
	0,          /* chip_id */
	0,          /* chip_version */
	0x70,       /* tuner_id */
	0x07,       /* i2c_speed */
	12000000,   /* xtal_hz: 12MHz */
	20250000,   /* adc_hz */
	0,          /* if_freq */
	false,      /* inversion */
	false       /* clock_out */
};
struct it917x_frontend {
	struct device *dev;
	const struct i2c_comm_master *i2c;
	u8 i2c_addr;	/* 7bit I2Cアドレス */

	struct it917x_config config;

	void *priv;
};

#ifdef __cplusplus
extern "C" {
#endif

int it917x_init(struct it917x_frontend *it917x);
int it917x_term(struct it917x_frontend *it917x);
int it917x_setup(struct it917x_frontend *it917x);
int it917x_set_frequency(struct it917x_frontend *it917x, u32 freq, u32 bandwidth);
int it917x_is_locked(struct it917x_frontend *it917x, bool *locked);
int it917x_read_cnr_raw(struct it917x_frontend *it917x, u32 *value);
int it917x_read_snr(struct it917x_frontend *it917x, u8 *value);

#ifdef __cplusplus
}
#endif

#endif
