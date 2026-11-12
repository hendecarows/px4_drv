// SPDX-License-Identifier: GPL-2.0-only
/*
 * PTX driver definitions for Digibest ISDBT6013 (DTV02-5T-P) device (isdbt6013_device.h)
 *
 */

#ifndef __ISDBT6013_DEVICE_H__
#define __ISDBT6013_DEVICE_H__

#include <linux/types.h>
#include <linux/device.h>
#include <linux/usb.h>

#include "ptx_chrdev.h"
#include "px4_card.h"
#include "it930x.h"
#include "it917x.h"

#define ISDBT6013_CHRDEV_MAX_NUM	5
#define ISDBT6013_CHRDEV_NUM	5

enum isdbt6013_model {
	ISDBT6013_MODEL = 0,
};

struct isdbt6013_device;

struct isdbt6013_chrdev {
	struct ptx_chrdev *chrdev;
	struct isdbt6013_device *parent;
	struct it917x_frontend it917x;
	bool locked;
	unsigned long capture_ready_at;
};

struct isdbt6013_device {
	struct mutex lock;
	struct kref kref;
	atomic_t available;
	struct device *dev;
	struct completion *quit_completion;
	struct px4_card *card;
	bool card_powered;
	unsigned int open_count;
	unsigned int streaming_count;
	struct ptx_chrdev_group *chrdev_group;
	int chrdevm_num;
	struct isdbt6013_chrdev chrdevm[ISDBT6013_CHRDEV_MAX_NUM];
	struct it930x_bridge it930x;
	void *stream_ctx;
};

int isdbt6013_device_init(struct isdbt6013_device *isdbt6013, struct device *dev,
		enum isdbt6013_model model,
		struct ptx_chrdev_context *chrdev_ctx,
		struct completion *quit_completion);
void isdbt6013_device_term(struct isdbt6013_device *isdbt6013);

#endif
