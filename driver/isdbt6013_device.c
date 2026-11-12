// SPDX-License-Identifier: GPL-2.0-only
/*
 * PTX driver for Digibest ISDBT6013 (DTV02-5T-P) device (isdbt6013_device.c)
 *
 */

#include "it917x.h"
#include "ptx_ioctl.h"
#include "isdbt6013_device.h"

#ifdef __linux__
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#endif

#include "px4_device_params.h"
#include "firmware.h"
#include "ts_sync.h"

#define ISDBT6013_DEVICE_TS_SYNC_COUNT	4
#define ISDBT6013_DEVICE_TS_SYNC_SIZE	(188 * ISDBT6013_DEVICE_TS_SYNC_COUNT)

struct isdbt6013_stream_context {
	struct ptx_chrdev *chrdev[ISDBT6013_CHRDEV_MAX_NUM];
	u8 remain_buf[ISDBT6013_DEVICE_TS_SYNC_SIZE];
	size_t remain_len;
};

static void isdbt6013_device_release(struct kref *kref);

static int isdbt6013_backend_set_power(struct isdbt6013_device *isdbt6013,
				bool state)
{
	int ret = 0;
	struct it930x_bridge *it930x = &isdbt6013->it930x;

	dev_dbg(isdbt6013->dev,
		"isdbt6013_backend_set_power: %s\n", (state) ? "true" : "false");

	if (!state && !atomic_read(&isdbt6013->available))
		return 0;

	if (state) {
		ret = it930x_write_gpio(it930x, 7, false);
		if (ret)
			return ret;

		msleep(80);

		ret = it930x_write_gpio(it930x, 2, true);
		if (ret)
			return ret;

		msleep(20);
	} else {
		it930x_write_gpio(it930x, 2, false);
		it930x_write_gpio(it930x, 7, true);
	}

	return 0;
}

static void isdbt6013_device_stream_process(struct ptx_chrdev **chrdev,
					u8 **buf, u32 *len)
{
	u8 *p = *buf;
	u32 remain = *len;

	while (likely(remain)) {
		u32 i;
		bool sync_remain = false;

		for (i = 0; i < ISDBT6013_DEVICE_TS_SYNC_COUNT; i++) {
			if (likely(((i + 1) * 188) <= remain)) {
				if (unlikely(!px4_ts_has_tagged_sync(p[i * 188])))
					break;
			} else {
				sync_remain = true;
				break;
			}
		}

		if (unlikely(sync_remain))
			break;

		if (unlikely(i < ISDBT6013_DEVICE_TS_SYNC_COUNT)) {
			p++;
			remain--;
			continue;
		}

		while (likely(remain >= 188 && px4_ts_has_tagged_sync(p[0]))) {
			u8 id = (p[0] & 0x70) >> 4;

			if (likely(id && id < 6)) {
				p[0] = 0x47;
				ptx_chrdev_put_stream(chrdev[id - 1], p, 188);
			}

			p += 188;
			remain -= 188;
		}
	}

	*buf = p;
	*len = remain;

	return;
}

static int isdbt6013_device_stream_handler(void *context, void *buf, u32 len)
{
	struct isdbt6013_stream_context *stream_ctx = context;
	u8 *p = buf;
	u32 remain = len;

	/* 同期探索後の端数も残す。新しい入力を二重に結合すると境界が壊れる。 */
	while (stream_ctx->remain_len && remain) {
		u8 *pending = stream_ctx->remain_buf;
		u32 pending_len, copy;

		copy = min_t(u32, ISDBT6013_DEVICE_TS_SYNC_SIZE - stream_ctx->remain_len, remain);
		memcpy(pending + stream_ctx->remain_len, p, copy);
		stream_ctx->remain_len += copy;
		p += copy;
		remain -= copy;
		if (stream_ctx->remain_len < ISDBT6013_DEVICE_TS_SYNC_SIZE)
			return 0;
		pending_len = stream_ctx->remain_len;
		isdbt6013_device_stream_process(stream_ctx->chrdev, &pending, &pending_len);
		memmove(stream_ctx->remain_buf, pending, pending_len);
		stream_ctx->remain_len = pending_len;
	}

	isdbt6013_device_stream_process(stream_ctx->chrdev, &p, &remain);

	if (unlikely(remain)) {
		memcpy(stream_ctx->remain_buf, p, remain);
		stream_ctx->remain_len = remain;
	}

	return 0;
}

static int isdbt6013_chrdev_init(struct ptx_chrdev *chrdev)
{
	dev_dbg(chrdev->parent->dev, "isdbt6013_chrdev_init\n");

	chrdev->params.system = PTX_ISDB_T_SYSTEM;
	return 0;
}

static int isdbt6013_chrdev_term(struct ptx_chrdev *chrdev)
{
	dev_dbg(chrdev->parent->dev, "isdbt6013_chrdev_term\n");
	return 0;
}

static int isdbt6013_chrdev_open(struct ptx_chrdev *chrdev)
{
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;
	struct isdbt6013_device *device = chrdevm->parent;
	int ret;

	chrdevm->locked = false;
	chrdevm->capture_ready_at = 0;
	mutex_lock(&device->lock);
	if (!atomic_read(&device->available)) {
		ret = -ENODEV;
		goto exit;
	}
	if (!device->open_count && !device->card_powered) {
		ret = isdbt6013_backend_set_power(device, true);
		if (ret)
			goto power_off;
	}
	ret = it917x_init(&chrdevm->it917x);
	if (ret)
		goto power_off;
	ret = it917x_setup(&chrdevm->it917x);
	if (ret) {
		it917x_term(&chrdevm->it917x);
		goto power_off;
	}
	device->open_count++;
	kref_get(&device->kref);
	mutex_unlock(&device->lock);
	return 0;

power_off:
	if (!device->open_count && !device->card_powered)
		isdbt6013_backend_set_power(device, false);
exit:
	mutex_unlock(&device->lock);
	return ret;
}

static int isdbt6013_chrdev_release(struct ptx_chrdev *chrdev)
{
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;
	struct isdbt6013_device *device = chrdevm->parent;

	mutex_lock(&device->lock);
	if (!device->open_count) {
		mutex_unlock(&device->lock);
		return -EALREADY;
	}
	it917x_term(&chrdevm->it917x);
	chrdevm->locked = false;
	chrdevm->capture_ready_at = 0;
	device->open_count--;
	/* 他チューナーの受信中は共有GPIOへ触れない。 */
	if (!device->open_count && !device->card_powered)
		isdbt6013_backend_set_power(device, false);
	if (kref_put(&device->kref, isdbt6013_device_release))
		return 0;
	mutex_unlock(&device->lock);
	return 0;
}

static int isdbt6013_chrdev_tune(struct ptx_chrdev *chrdev,
			       struct ptx_tune_params *params)
{
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;

	if (!atomic_read(&chrdevm->parent->available))
		return -ENODEV;
	if (params->system != PTX_ISDB_T_SYSTEM)
		return -EINVAL;
	chrdevm->locked = false;
	chrdevm->capture_ready_at = 0;
	return it917x_set_frequency(&chrdevm->it917x, params->freq, params->bandwidth);
}

static int isdbt6013_chrdev_check_lock(struct ptx_chrdev *chrdev, bool *locked)
{
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;
	int ret;

	*locked = false;
	chrdevm->locked = false;
	chrdevm->capture_ready_at = 0;
	if (!atomic_read(&chrdevm->parent->available))
		return -ENODEV;
	ret = it917x_is_locked(&chrdevm->it917x, locked);
	chrdevm->locked = !ret && *locked;
	if (chrdevm->locked) {
		/* ロック後もデインターリーブの整定が残るため、実機で確認した350msを確保する。 */
		chrdevm->capture_ready_at = jiffies + msecs_to_jiffies(350);
		/* 継続選局ではstart_captureを通らないため、選局完了前に整定を待つ。 */
		if (chrdev->streaming) {
			msleep(350);
			if (!atomic_read(&chrdevm->parent->available)) {
				*locked = false;
				chrdevm->locked = false;
				chrdevm->capture_ready_at = 0;
				return -ENODEV;
			}
		}
	}
	return ret;
}

static int isdbt6013_chrdev_start_capture(struct ptx_chrdev *chrdev)
{
	int ret = 0;
	struct ptx_chrdev_group *chrdev_group = chrdev->parent;
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;
	struct isdbt6013_device *isdbt6013 = chrdevm->parent;
	long settle_remaining;

	dev_dbg(isdbt6013->dev,
		"isdbt6013_chrdev_start_capture %u:%u\n",
		chrdev_group->id, chrdev->id);

	if (!chrdevm->locked)
		return -EAGAIN;
	/* 他入力の配信・開始停止を妨げないよう、共有ロックの取得前に整定を待つ。 */
	settle_remaining = chrdevm->capture_ready_at - jiffies;
	if (settle_remaining > 0)
		msleep(jiffies_to_msecs(settle_remaining) + 1);
	mutex_lock(&isdbt6013->lock);
	if (!atomic_read(&isdbt6013->available)) {
		ret = -ENODEV;
		goto exit;
	}

	if (!isdbt6013->streaming_count) {
		struct isdbt6013_stream_context *stream_ctx = isdbt6013->stream_ctx;

		ret = it930x_purge_psb(&isdbt6013->it930x,
				       px4_device_params.psb_purge_timeout);
		if (ret) {
			dev_err(isdbt6013->dev,
				"isdbt6013_chrdev_start_capture %u:%u: it930x_purge_psb() failed. (ret: %d)\n",
				chrdev_group->id, chrdev->id, ret);
			goto exit;
		}

		stream_ctx->remain_len = 0;

		ret = itedtv_bus_start_streaming(&isdbt6013->it930x.bus,
						 isdbt6013_device_stream_handler,
						 stream_ctx);
		if (ret) {
			dev_err(isdbt6013->dev,
				"isdbt6013_chrdev_start_capture %u:%u: itedtv_bus_start_streaming() failed. (ret: %d)\n",
				chrdev_group->id, chrdev->id, ret);
			goto exit;
		}
	}

	isdbt6013->streaming_count++;

	dev_dbg(isdbt6013->dev,
		"isdbt6013_chrdev_start_capture %u:%u: streaming_count: %u\n",
		chrdev_group->id, chrdev->id, isdbt6013->streaming_count);

exit:
	mutex_unlock(&isdbt6013->lock);
	return ret;
}

static int isdbt6013_chrdev_stop_capture(struct ptx_chrdev *chrdev)
{
	struct ptx_chrdev_group *chrdev_group = chrdev->parent;
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;
	struct isdbt6013_device *isdbt6013 = chrdevm->parent;

	dev_dbg(isdbt6013->dev,
		"isdbt6013_chrdev_stop_capture %u:%u\n",
		chrdev_group->id, chrdev->id);

	mutex_lock(&isdbt6013->lock);

	if (!isdbt6013->streaming_count) {
		mutex_unlock(&isdbt6013->lock);
		return -EALREADY;
	}

	isdbt6013->streaming_count--;
	if (!isdbt6013->streaming_count) {
		dev_dbg(isdbt6013->dev,
			"isdbt6013_chrdev_stop_capture %u:%u: stopping...\n",
			chrdev_group->id, chrdev->id);
		itedtv_bus_stop_streaming(&isdbt6013->it930x.bus);
	} else {
		dev_dbg(isdbt6013->dev,
			"isdbt6013_chrdev_stop_capture %u:%u: streaming_count: %u\n",
			chrdev_group->id, chrdev->id, isdbt6013->streaming_count);
	}

	mutex_unlock(&isdbt6013->lock);
	return 0;
}

static int isdbt6013_chrdev_set_capture(struct ptx_chrdev *chrdev, bool status)
{
	return (status) ? isdbt6013_chrdev_start_capture(chrdev)
			: isdbt6013_chrdev_stop_capture(chrdev);
}

static int isdbt6013_chrdev_read_cnr_raw(struct ptx_chrdev *chrdev, u32 *value)
{
	struct isdbt6013_chrdev *chrdevm = chrdev->priv;

	if (!value)
		return -EINVAL;
	*value = 0;
	if (!atomic_read(&chrdevm->parent->available))
		return -ENODEV;
	if (chrdev->current_system != PTX_ISDB_T_SYSTEM)
		return -EINVAL;
	return it917x_read_cnr_raw(&chrdevm->it917x, value);
}

static struct ptx_chrdev_operations isdbt6013_chrdev_ops = {
	.init = isdbt6013_chrdev_init,
	.term = isdbt6013_chrdev_term,
	.open = isdbt6013_chrdev_open,
	.release = isdbt6013_chrdev_release,
	.tune = isdbt6013_chrdev_tune,
	.check_lock = isdbt6013_chrdev_check_lock,
	.set_stream_id = NULL,
	.set_lnb_voltage = NULL,
	.set_capture = isdbt6013_chrdev_set_capture,
	.read_signal_strength = NULL,
	.read_cnr = NULL,
	.read_cnr_raw = isdbt6013_chrdev_read_cnr_raw,
};

static const struct {
	u8 i2c_addr;
	u8 i2c_bus;
	u8 port_number;
} isdbt6013_device_params[] = {
	{ 0x1c, 3, 0 },
	{ 0x1c, 1, 1 },
	{ 0x1e, 1, 2 },
	{ 0x1e, 3, 3 },
	{ 0x1f, 3, 4 },
};

static int isdbt6013_device_load_config(struct isdbt6013_device *isdbt6013)
{
	int i;
	struct it930x_bridge *it930x = &isdbt6013->it930x;

	for (i = 0; i < ISDBT6013_CHRDEV_NUM; i++) {
		struct it930x_stream_input *input = &it930x->config.input[i];

		input->enable = true;
		input->is_parallel = false;
		input->port_number = isdbt6013_device_params[i].port_number;
		input->slave_number = i;
		input->i2c_bus = isdbt6013_device_params[i].i2c_bus;
		input->i2c_addr = isdbt6013_device_params[i].i2c_addr;
		input->packet_len = 188;
		input->sync_byte = ((i + 1) << 4) | 0x07;

		isdbt6013->chrdevm[i].it917x.dev = isdbt6013->dev;
		isdbt6013->chrdevm[i].it917x.i2c = &it930x->i2c_master[input->i2c_bus - 1];
		isdbt6013->chrdevm[i].it917x.i2c_addr = input->i2c_addr;
	}

	return 0;
}

/* px4_card が device->lock を保持した状態で呼ぶ。 */
static int isdbt6013_device_set_card_power(void *context, bool on)
{
	struct isdbt6013_device *device = context;
	int ret = 0;

	/* チューナー利用中は共有GPIOを操作せず、カード側の参照だけを更新する。 */
	if (!device->open_count) {
		ret = isdbt6013_backend_set_power(device, on);
		/* GPIO操作の途中で投入に失敗しても、未使用基板を通電したまま残さない。 */
		if (on && ret)
			isdbt6013_backend_set_power(device, false);
	}
	/* 停止は成否にかかわらず記録し、投入だけは成功を確認する。 */
	if (!on || !ret)
		device->card_powered = on;
	return ret;
}

int isdbt6013_device_init(struct isdbt6013_device *device, struct device *dev,
			 enum isdbt6013_model model,
			 struct ptx_chrdev_context *chrdev_ctx,
			 struct completion *quit_completion)
{
	struct it930x_bridge *it930x;
	struct itedtv_bus *bus;
	struct isdbt6013_stream_context *stream_ctx;
	struct ptx_chrdev_config config[ISDBT6013_CHRDEV_NUM] = { 0 };
	struct ptx_chrdev_group_config group_config = { 0 };
	int ret, i;

	if (!device || !dev || !chrdev_ctx || !quit_completion || model != ISDBT6013_MODEL)
		return -EINVAL;
	get_device(dev);
	mutex_init(&device->lock);
	kref_init(&device->kref);
	device->dev = dev;
	device->quit_completion = quit_completion;
	device->card = NULL;
	device->card_powered = false;
	device->chrdevm_num = ISDBT6013_CHRDEV_NUM;
	stream_ctx = kzalloc(sizeof(*stream_ctx), GFP_KERNEL);
	if (!stream_ctx) {
		ret = -ENOMEM;
		goto fail;
	}
	device->stream_ctx = stream_ctx;
	it930x = &device->it930x;
	bus = &it930x->bus;
	ret = itedtv_bus_init(bus);
	if (ret) {
		dev_err(dev,
			"isdbt6013_device_init: itedtv_bus_init() failed. (ret: %d)\n",
			ret);
		goto fail_bus;
	}

	ret = it930x_init(it930x);
	if (ret) {
		dev_err(dev,
			"isdbt6013_device_init: it930x_init() failed. (ret: %d)\n",
			ret);
		goto fail_bridge;
	}

	ret = it930x_raise(it930x);
	if (ret) {
		dev_err(dev,
			"isdbt6013_device_init: it930x_raise() failed. (ret: %d)\n",
			ret);
		goto fail_device;
	}

	ret = isdbt6013_device_load_config(device);
	if (ret)
		goto fail_device;

	ret = it930x_load_firmware(it930x, IT930X_FIRMWARE_FILENAME);
	if (ret) {
		dev_err(dev,
			"isdbt6013_device_init: it930x_load_firmware() failed. (ret: %d)\n",
			ret);
		goto fail_device;
	}

	ret = it930x_init_warm(it930x);
	if (ret) {
		dev_err(dev,
			"isdbt6013_device_init: it930x_init_warm() failed. (ret: %d)\n",
			ret);
		goto fail_device;
	}

	ret = it930x_set_gpio_mode(it930x, 7, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail_device;

	ret = it930x_write_gpio(it930x, 7, true);
	if (ret)
		goto fail_device;

	ret = it930x_set_gpio_mode(it930x, 2, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail_device;

	ret = it930x_write_gpio(it930x, 2, false);
	if (ret)
		goto fail_device;

	if (px4_device_params.discard_null_packets) {
		struct it930x_pid_filter filter = {
			.block = true,
			.num = 1,
			.pid = { 0x1fff },
		};
		for (i = 0; i < ISDBT6013_CHRDEV_NUM; i++) {
			ret = it930x_set_pid_filter(it930x, i, &filter);
			if (ret)
				goto fail_device;
		}
	}

	for (i = 0; i < ISDBT6013_CHRDEV_NUM; i++) {
		device->chrdevm[i].parent = device;
		config[i].system_cap = PTX_ISDB_T_SYSTEM;
		config[i].ops = &isdbt6013_chrdev_ops;
		config[i].ringbuf_size = 188 * px4_device_params.tsdev_max_packets;
		config[i].ringbuf_threshold_size = config[i].ringbuf_size / 10;
		config[i].priv = &device->chrdevm[i];
	}
	group_config.owner_kref = &device->kref;
	group_config.owner_kref_release = isdbt6013_device_release;
	group_config.chrdev_num = ISDBT6013_CHRDEV_NUM;
	group_config.chrdev_config = config;
	ret = ptx_chrdev_context_add_group(chrdev_ctx, dev, &group_config, &device->chrdev_group);
	if (ret)
		goto fail_device;
	for (i = 0; i < ISDBT6013_CHRDEV_NUM; i++) {
		device->chrdevm[i].chrdev = &device->chrdev_group->chrdev[i];
		stream_ctx->chrdev[i] = &device->chrdev_group->chrdev[i];
	}
	atomic_set(&device->available, 1);
	{
		struct px4_card_backend card_backend = {
			.it930x = it930x,
			.lock = &device->lock,
			.available = &device->available,
			.set_power = isdbt6013_device_set_card_power,
			.context = device,
		};

		/* カード登録の失敗で既存のチューナー利用まで妨げない。 */
		ret = px4_card_register(dev, &card_backend, &device->card);
		if (ret)
			dev_warn(dev, "Card reader registration failed: %d\n", ret);
	}
	return 0;

fail_device:
	it930x_term(it930x);
fail_bridge:
	itedtv_bus_term(bus);
fail_bus:
	kfree(device->stream_ctx);
fail:
	mutex_destroy(&device->lock);
	put_device(dev);
	return ret;
}

static void isdbt6013_device_release(struct kref *kref)
{
	struct isdbt6013_device *isdbt6013 = container_of(kref,
						  struct isdbt6013_device, kref);

	dev_dbg(isdbt6013->dev, "isdbt6013_device_release\n");

	it930x_term(&isdbt6013->it930x);
	itedtv_bus_term(&isdbt6013->it930x.bus);

	kfree(isdbt6013->stream_ctx);

	mutex_destroy(&isdbt6013->lock);
	put_device(isdbt6013->dev);

	complete(isdbt6013->quit_completion);
	return;
}

void isdbt6013_device_term(struct isdbt6013_device *device)
{
	/* カード操作を止めてから登録を外し、機種オブジェクトへの参照を切り離す。 */
	mutex_lock(&device->lock);
	atomic_set(&device->available, 0);
	mutex_unlock(&device->lock);
	px4_card_unregister(device->card);
	device->card = NULL;
	ptx_chrdev_group_destroy(device->chrdev_group);
	kref_put(&device->kref, isdbt6013_device_release);
}
