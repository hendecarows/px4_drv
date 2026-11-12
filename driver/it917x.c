// SPDX-License-Identifier: GPL-2.0-only
/*
 * ITE IT917x driver (it917x.c)
 *
 * it930x.c の it930x_ctrl_msg() 系ロジックを、I2Cトンネル越しに置き換えて再利用する。
 *
 *  outer: I2C_WRITE(bus, addr, inner_tx_frame)
 *  outer: I2C_READ(bus, addr, len = inner_data_len + 5)  -> inner_rx_frame
 *
 *  inner_tx_frame = [len-1][cmd_hi][cmd_lo][seq][payload...][chk_hi][chk_lo]
 *  inner_rx_frame = [len-1][seq   ][result][data...   ][chk_hi][chk_lo]
 *
 */

#include "print_format.h"
#include "firmware.h"
#include "it917x.h"

#ifdef __linux__
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/firmware.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#endif

#if defined(_WIN32) || defined(_WIN64)
/* Linux の待機精度を他機種へ適用せず、IT917x の校正だけを単調時計で制限する。 */
#define min_t(type, a, b) ((type)(a) < (type)(b) ? (type)(a) : (type)(b))
#endif

struct it917x_priv {
	struct mutex lock;
	u8 seq;
	u8 buf[256];
	u32 fxtal_khz;
	u32 fdiv;
	u32 f_boundary;
	u8 clock_mode;
};

struct it917x_ctrl_buf {
	u8 *buf;
	u8 len;
};

struct it917x_reg_val {
	enum it917x_proc proc;
	u32 reg;
	u8 val;
};

static inline u8 it917x_reg_length(u32 reg)
{
	if (reg & 0xff000000)
		return 4;
	if (reg & 0x00ff0000)
		return 3;
	if (reg & 0x0000ff00)
		return 2;
	return 1;
}

/* it930x_calc_checksum() と同一アルゴリズム */
static u16 it917x_calc_checksum(void *buf, size_t len)
{
	int i;
	u8 *b;
	u16 c = 0;

	i = (int)(len / 2);
	b = buf;

	while (i--) {
		c += ((b[0] << 8) | (b[1]));
		b += 2;
	}

	if (len % 2)
		c += (b[0] << 8);

	return ~c;
}

/*
 * it917x_ctrl_msg() - 内側コマンドフレームをI2Cトンネル越しに送受信する
 *
 * @cmd:    IT917X_CMD_*
 * @wbuf:   送信ペイロード (NULL可)
 * @rbuf:   受信データ格納先。rbuf->len に「期待するデータ長」を事前に
 *          セットしておくこと(I2C_READで要求するバイト数の算出に使うため、
 *          it930x_ctrl_msg()と違い、事前に長さが分かっている必要がある)。
 * @result: 応答のresultバイトを格納 (NULL可)
 */
static int it917x_ctrl_msg(struct it917x_frontend *it917x,
			u16 cmd,
			struct it917x_ctrl_buf *wbuf,
			struct it917x_ctrl_buf *rbuf,
			u8 *result)
{
	int ret;
	struct it917x_priv *priv = it917x->priv;
	struct i2c_comm_request req;
	u8 *buf, len, seq;
	u16 csum, csum2;
	int rlen;

	/* IT930x の外側 I2C_WRITE は247バイト、I2C_READ は251バイトまで。 */
	if ((wbuf && (wbuf->len > 241 || (wbuf->len && !wbuf->buf))) ||
	    (rbuf && rbuf->len > 246))
		return -EINVAL;

	mutex_lock(&priv->lock);

	buf = priv->buf;
	len = 4 + 2;
	if (wbuf)
		len += wbuf->len;
	seq = priv->seq++;

	buf[0] = len - 1;
	buf[1] = ((cmd >> 8) & 0xff);
	buf[2] = (cmd & 0xff);
	buf[3] = seq;
	if (wbuf && wbuf->buf)
		memcpy(&buf[4], wbuf->buf, wbuf->len);

	csum = it917x_calc_checksum(&buf[1], (size_t)len - 1 - 2);
	buf[len - 2] = ((csum >> 8) & 0xff);
	buf[len - 1] = (csum & 0xff);
#ifdef __linux__
	dev_dbg(it917x->dev, "IT917x addr=%02x TX %*ph\n", it917x->i2c_addr, len, buf);
#endif

	/* outer: I2C_WRITE で inner tx frame を送信 */
	req.req = I2C_WRITE_REQUEST;
	req.addr = it917x->i2c_addr;
	req.data = buf;
	req.len = len;

	ret = i2c_comm_master_request(it917x->i2c, &req, 1);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_ctrl_msg: i2c write failed. (cmd: 0x%04x, ret: %d)\n",
			cmd, ret);
		goto exit;
	}

	/*
	 * outer: I2C_READ で inner rx frame を受信する。
	 * 要求バイト数 = 期待データ長 + 5 (len/seq/result/checksum2)。
	 * rbuf が NULL の場合はデータ無し応答(result確認のみ)として扱う。
	 */
	rlen = (rbuf ? rbuf->len : 0) + 5;

	req.req = I2C_READ_REQUEST;
	req.addr = it917x->i2c_addr;
	req.data = buf;
	req.len = rlen;

	ret = i2c_comm_master_request(it917x->i2c, &req, 1);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_ctrl_msg: i2c read failed. (cmd: 0x%04x, ret: %d)\n",
			cmd, ret);
		goto exit;
	}
#ifdef __linux__
	dev_dbg(it917x->dev, "IT917x addr=%02x RX %*ph\n", it917x->i2c_addr, rlen, buf);
#endif

	if (buf[0] != rlen - 1) {
		dev_err(it917x->dev,
			"it917x_ctrl_msg: incorrect response length. (expected: %d, received: %u)\n",
			rlen, buf[0] + 1);
		ret = -EBADMSG;
		goto exit;
	}

	csum = it917x_calc_checksum(&buf[1], (size_t)rlen - 1 - 2);
	csum2 = ((buf[rlen - 2] << 8) | buf[rlen - 1]);
	if (csum != csum2) {
		dev_err(it917x->dev,
			"it917x_ctrl_msg: checksum is incorrect. (0x%04x, 0x%04x)\n",
			csum, csum2);
		ret = -EBADMSG;
		goto exit;
	}

	if (buf[1] != seq) {
		dev_err(it917x->dev,
			"it917x_ctrl_msg: sequence number is incorrect. (tx: 0x%02x, rx: 0x%02x)\n",
			seq, buf[1]);
		ret = -EBADMSG;
		goto exit;
	}

	if (buf[2]) {
		dev_err(it917x->dev,
			"it917x_ctrl_msg: error returned. (result: %u)\n",
			buf[2]);
		ret = -EIO;
	} else if (rbuf && rbuf->buf) {
		u8 dlen = rlen - 3 - 2;

		if (dlen > rbuf->len)
			dlen = rbuf->len;
		memcpy(rbuf->buf, &buf[3], dlen);
	}

	if (result)
		*result = buf[2];

exit:
	mutex_unlock(&priv->lock);

	return ret;
}

static inline u16 it917x_proc_cmd(enum it917x_proc proc, u16 base_cmd)
{
	return proc | base_cmd;
}

static inline enum it917x_proc it917x_addr_proc(u32 addr)
{
	return (addr & 0x800000) ? IT917X_PROC_OFDM : IT917X_PROC_LINK;
}

static int it917x_read_regs(struct it917x_frontend *it917x, enum it917x_proc proc,
			u32 reg, u8 *rbuf, u8 len)
{
	u8 buf[6];
	struct it917x_ctrl_buf wb, rb;

	if (!rbuf || !len || len > 246)
		return -EINVAL;

	buf[0] = len;
	buf[1] = (proc == IT917X_PROC_OFDM) ? 2 : it917x_reg_length(reg);
	buf[2] = ((reg >> 24) & 0xff);
	buf[3] = ((reg >> 16) & 0xff);
	buf[4] = ((reg >> 8) & 0xff);
	buf[5] = (reg & 0xff);

	wb.buf = buf;
	wb.len = 6;

	rb.buf = rbuf;
	rb.len = len;	/* 期待データ長。I2C_READの要求長算出に使われる */

	return it917x_ctrl_msg(it917x,
			it917x_proc_cmd(proc, IT917X_CMD_REG_READ),
			&wb, &rb, NULL);
}

static int it917x_read_reg(struct it917x_frontend *it917x, enum it917x_proc proc, u32 reg, u8 *val)
{
	return it917x_read_regs(it917x, proc, reg, val, 1);
}

static int it917x_write_regs(struct it917x_frontend *it917x, enum it917x_proc proc,
			u32 reg, const u8 *wbuf, u8 len)
{
	u8 buf[250];
	struct it917x_ctrl_buf wb, rb;

	if (!wbuf || !len || len > 235)
		return -EINVAL;

	buf[0] = len;
	buf[1] = (proc == IT917X_PROC_OFDM) ? 2 : it917x_reg_length(reg);
	buf[2] = ((reg >> 24) & 0xff);
	buf[3] = ((reg >> 16) & 0xff);
	buf[4] = ((reg >> 8) & 0xff);
	buf[5] = (reg & 0xff);
	memcpy(&buf[6], wbuf, len);

	wb.buf = buf;
	wb.len = 6 + len;

	/* REG_WRITEの応答はデータ無し(result確認のみ) */
	rb.buf = NULL;
	rb.len = 0;

	return it917x_ctrl_msg(it917x,
			it917x_proc_cmd(proc, IT917X_CMD_REG_WRITE),
			&wb, &rb, NULL);
}

static int it917x_write_reg(struct it917x_frontend *it917x, enum it917x_proc proc, u32 reg, u8 val)
{
	return it917x_write_regs(it917x, proc, reg, &val, 1);
}

static int it917x_write_reg_val(struct it917x_frontend *it917x, const struct it917x_reg_val *reg_val, int count)
{
	int ret = 0;
	int i;

	for (i = 0; i < count; i++) {
		ret = it917x_write_reg(it917x, reg_val[i].proc, reg_val[i].reg, reg_val[i].val);

		if (ret) {
			dev_err(it917x->dev,
				"it917x_write_reg_val: write failed. (proc: %d, reg: 0x%04x, val: 0x%02x, ret: %d)\n",
				reg_val[i].proc, reg_val[i].reg, reg_val[i].val, ret);
			break;
		}
	}

	return ret;
}

static int it917x_write_reg_mask(struct it917x_frontend *it917x, enum it917x_proc proc,
			u32 reg, u8 val, u8 mask)
{
	int ret = 0;
	u8 tmp;

	if (!mask)
		return -EINVAL;

	if (mask != 0xff) {
		ret = it917x_read_reg(it917x, proc, reg, &tmp);
		if (ret)
			return ret;

		tmp &= ~mask;
		tmp |= (val & mask);
	} else {
		tmp = val;
	}

	return it917x_write_reg(it917x, proc, reg, tmp);
}

int it917x_init(struct it917x_frontend *it917x)
{
	struct it917x_priv *priv;

	if (!it917x->i2c || !it917x->dev)
		return -EINVAL;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	it917x->config = it917x_default_config;
	mutex_init(&priv->lock);
	priv->seq = 0;
	it917x->priv = priv;

	return 0;
}

int it917x_term(struct it917x_frontend *it917x)
{
	struct it917x_priv *priv = it917x->priv;

	if (priv) {
		mutex_destroy(&priv->lock);
		kfree(priv);
		it917x->priv = NULL;
	}

	return 0;
}

static int it917x_read_chip_id(struct it917x_frontend *it917x, u16 *chip_id, u8 *chip_version)
{
	int ret = 0;
	u8 buf[3];
	u32 id;

	if (!chip_id || !chip_version)
		return -EINVAL;

	// 0x1222:  chip version
	// 0x1223,0x1224:  chip id
	ret = it917x_read_regs(it917x, IT917X_PROC_LINK, 0x1222, buf, 3);
	if (ret)
		return ret;

	id = ((u16)buf[2] << 8) | buf[1];

	dev_dbg(it917x->dev, "it917x_get_chip_id: id=0x%04x version=%02x\n", id, buf[0]);

	*chip_id = id;
	*chip_version = buf[0];

	return 0;
}

static int it917x_read_xtal_freq(struct it917x_frontend *it917x, u32 *xtal_hz, u32 *adc_hz)
{
	int ret = 0;
	u8 val;
	u32 xtal, adc;

	if (!xtal_hz || !adc_hz)
		return -EINVAL;

	ret = it917x_read_reg(it917x, IT917X_PROC_LINK, 0xd800, &val);
	if (ret)
		return ret;

	val &= 0x0f;

	switch (val) {
	case 0x00:
		xtal = 12000000;
		adc = 20250000;
		break;
	case 0x01:
		xtal = 20480000;
		adc = 20480000;
		break;
	default:
		dev_err(it917x->dev, "it917x_read_xtal: unknown xtal value: %u\n", val);
		return -EINVAL;
	}
	dev_dbg(it917x->dev, "it917x_read_xtal: xtal=%u Hz adc=%u Hz\n", xtal, adc);

	*xtal_hz = xtal;
	*adc_hz = adc;

	return 0;
}


static int it917x_read_firmware_version(struct it917x_frontend *it917x,
			u32 *fw_version)
{
	int ret = 0;
	u8 buf[4];
	struct it917x_ctrl_buf wb, rb;

	buf[0] = 1;

	wb.buf = buf;
	wb.len = 1;

	rb.buf = buf;
	rb.len = 4;

	dev_dbg(it917x->dev,"it917x_read_firmware_version\n");
	ret = it917x_ctrl_msg(it917x, IT917X_CMD_QUERYINFO, &wb, &rb, NULL);
	if (ret)
		return ret;

	if (fw_version)
		*fw_version = ((u32)buf[0] << 24) | ((u32)buf[1] << 16)
			| ((u32)buf[2] << 8) | buf[3];

	return 0;
}

static int it917x_firmware_block_length(const u8 *data, size_t size, size_t *length)
{
	size_t header, payload = 0;
	unsigned int i;

	if (size < 4 || data[0] != 3 || !data[3])
		return -EINVAL;
	header = 4 + (size_t)data[3] * 3;
	if (header > size)
		return -EINVAL;
	for (i = 0; i < data[3]; i++)
		payload += data[6 + i * 3];
	if (!payload || header + payload > size || header + payload > 241)
		return -EINVAL;
	*length = header + payload;
	return 0;
}

static int it917x_load_firmware(struct it917x_frontend *it917x, const char *filename)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u32 fw_version;
	size_t i, n, len = 0;
	struct it917x_ctrl_buf wb, rb;

	if (!filename)
		return -EINVAL;

	dev_dbg(it917x->dev,"it917x_load_firmware\n");

	ret = it917x_read_firmware_version(it917x, &fw_version);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_load_firmware: it917x_read_firmware_version() failed. 1 (ret: %d)\n",
			ret);
		return ret;
	}
	dev_dbg(it917x->dev,"it917x_read_firmware_version version: %08x\n", fw_version);

	if (fw_version) {
		dev_info(it917x->dev,
			 "Firmware is already loaded. version: %d.%d.%d.%d\n",
			 (fw_version >> 24) & 0xff, (fw_version >> 16) & 0xff,
			 (fw_version >> 8) & 0xff, fw_version & 0xff);
		return ret;
	}

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0xf103, it917x->config.i2c_speed);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_load_firmware: it917x_write_reg(0xf103) failed. (ret: %d)\n",
			ret);
		return ret;
	}

	ret = request_firmware(&fw, filename, it917x->dev);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_load_firmware: request_firmware() failed. (ret: %d)\n",
			ret);
		dev_err(it917x->dev,
			"Couldn't load firmware from the file.\n");
		return ret;
	}

	n = fw->size;
	if (!n) {
		ret = -EINVAL;
		goto exit;
	}
	/* 壊れた後続ブロックでアップロード途中の状態を残さないよう、先に全体を検査する。 */
	for (i = 0; i < n; i += len) {
		ret = it917x_firmware_block_length(fw->data + i, n - i, &len);
		if (ret)
			goto exit;
	}

	for (i = 0; i < n; i += len) {
		const u8 *p = &fw->data[i];
		unsigned j, m;
		size_t header;

		len = 0;

		if (n - i < 4 || p[0] != 0x03) {
			dev_err(it917x->dev,
				"it917x_load_firmware: Invalid firmware block was found. Abort. (ofs: %zx)\n",
				i);
			ret = -ECANCELED;
			goto exit;
		}

		m = p[3];
		header = 4 + (size_t)m * 3;
		if (!m || header > n - i) {
			ret = -EINVAL;
			goto exit;
		}
		for (j = 0; j < m; j++)
			len += p[6 + (j * 3)];

		if (!len) {
			dev_warn(it917x->dev,
				 "it917x_load_firmware: No data in the block. (ofs: %zx)\n",
				 i);
			ret = -EINVAL;
			goto exit;
		}

		len += header;
		if (len > n - i || len > 241) {
			ret = -EINVAL;
			goto exit;
		}

		/* firmware block を送信する */

		wb.buf = (u8 *)p;
		wb.len = (u8)len;

		rb.buf = NULL;
		rb.len = 0;

		ret = it917x_ctrl_msg(it917x, IT917X_CMD_FW_SCATTER_WRITE,
				      &wb, &rb, NULL);
		if (ret) {
			dev_err(it917x->dev,
				"it917x_load_firmware: it917x_ctrl_msg(IT917X_CMD_FW_SCATTER_WRITE) failed. (ofs: %zx, ret: %d)\n",
				i, ret);
			goto exit;
		}
	}

	ret = it917x_ctrl_msg(it917x, IT917X_CMD_BOOT, NULL, NULL, NULL);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_load_firmware: it917x_ctrl_msg(IT917X_CMD_BOOT) failed. (ret: %d)\n",
			ret);
		goto exit;
	}

	msleep(10);
	ret = it917x_read_firmware_version(it917x, &fw_version);
	if (ret) {
		dev_err(it917x->dev,
			"it917x_load_firmware: it917x_read_firmware_version() failed. 2 (ret: %d)\n",
			ret);
		goto exit;
	}

	if (!fw_version) {
		ret = -EIO;
		goto exit;
	}

	dev_info(it917x->dev,
		 "it917x firmware loaded. version: %d.%d.%d.%d\n",
		 (fw_version >> 24) & 0xff, (fw_version >> 16) & 0xff,
		 (fw_version >> 8) & 0xff, fw_version & 0xff);

exit:
	release_firmware(fw);

	return ret;
}

static int it917x_get_divider(u32 a, u32 b, u32 x, u32 *answer)
{
	u32 ans = 0;
	u32 c = 0;
	u32 i = 0;

	if (!answer || b == 0) {
		return -EINVAL;
	}

	if (a >= b) {
		c = a / b;
		a = a % b;
	}

	for (i = 0; i < x; i++) {
		ans <<= 1;
		a <<= 1;

		if (a >= b) {
			ans += 1;
			a -= b;
		}
	}

	*answer = (c << x) + ans;

	return 0;
}

static int it917x_get_divider_xtal(u32 freq, u8 *buf, int size)
{
	int ret = 0;
	u32 divider;

	if (!buf || size < 4)
		return -EINVAL;

	ret = it917x_get_divider(freq, 1000000, 19, &divider);
	if (ret)
		return ret;
	buf[0] = (u8)(divider & 0xff);
	buf[1] = (u8)((divider >> 8) & 0xff);
	buf[2] = (u8)((divider >> 16) & 0xff);
	buf[3] = (u8)((divider >> 24) & 0xff);

	return 0;
}

static int it917x_get_divider_adc(u32 freq, u8 *buf, int size)
{
	int ret = 0;
	u32 divider;

	if (!buf || size < 3)
		return -EINVAL;

	ret = it917x_get_divider(freq, 1000000, 19, &divider);
	if (ret)
		return ret;
	buf[0] = (u8)(divider & 0xff);
	buf[1] = (u8)((divider >> 8) & 0xff);
	buf[2] = (u8)((divider >> 16) & 0xff);

	return 0;
}

/*
 * IT917x initialization data.
 *
 */

static const struct it917x_script_entry {
	u16 reg;
	u8 len;
	u8 data[20];
} it917x_script[] = {
	{ 0x0037, 2, { 0x10, 0x27 } },
	{ 0x003a, 2, { 0x10, 0x27 } },
	{ 0x004b, 1, { 0x01 } },
	{ 0x004d, 1, { 0x01 } },
	{ 0x0051, 1, { 0x01 } },
	{ 0x0055, 2, { 0x01, 0x00 } },
	{ 0x0065, 1, { 0x50 } },
	{ 0x0070, 1, { 0x0a } },
	{ 0x007e, 1, { 0x04 } },
	{ 0x0081, 1, { 0x0a } },
	{ 0x008a, 9, { 0x01, 0x07, 0x07, 0x06, 0x01, 0x20, 0x2c, 0x01, 0x06 } },
	{ 0x0099, 1, { 0x01 } },
	{ 0x009f, 2, { 0xe1, 0xcf } },
	{ 0x00a3, 1, { 0x01 } },
	{ 0x00a5, 2, { 0x01, 0x01 } },
	{ 0x00a9, 2, { 0x00, 0x01 } },
	{ 0x00b0, 1, { 0x01 } },
	{ 0x00c2, 1, { 0x05 } },
	{ 0x00c6, 1, { 0x19 } },
	{ 0x00f8, 4, { 0x10, 0x27, 0x10, 0x27 } },
	{ 0x0101, 4, { 0x10, 0x27, 0x10, 0x27 } },
	{ 0x01b5, 2, { 0x2b, 0x0b } },
	{ 0x01c6, 2, { 0x01, 0x01 } },
	{ 0x01ef, 4, { 0x01, 0x01, 0x01, 0x01 } },
	{ 0x01f5, 3, { 0x01, 0x01, 0x18 } },
	{ 0xf000, 1, { 0x0f } },
	{ 0xf02b, 1, { 0x00 } },
	{ 0xf064, 4, { 0x03, 0xf9, 0x03, 0x01 } },
	{ 0xf06f, 2, { 0xe0, 0x03 } },
	{ 0xf072, 2, { 0x0f, 0x03 } },
	{ 0xf078, 1, { 0x00 } },
	{ 0xf087, 1, { 0x00 } },
	{ 0xf09b, 6, { 0x3f, 0x00, 0x20, 0x00, 0x0c, 0x00 } },
	{ 0xf130, 1, { 0x04 } },
	{ 0xf132, 1, { 0x04 } },
	{ 0xf144, 1, { 0x1a } },
	{ 0xf146, 1, { 0x00 } },
	{ 0xf14a, 1, { 0x01 } },
	{ 0xf14c, 2, { 0x00, 0x00 } },
	{ 0xf14f, 1, { 0x04 } },
	{ 0xf158, 1, { 0x7f } },
	{ 0xf15a, 2, { 0x00, 0x08 } },
	{ 0xf15d, 3, { 0x03, 0x05, 0x02 } },
	{ 0xf163, 1, { 0x05 } },
	{ 0xf166, 4, { 0x01, 0x40, 0x0f, 0x08 } },
	{ 0xf17a, 2, { 0x00, 0x00 } },
	{ 0xf183, 1, { 0x01 } },
	{ 0xf185, 1, { 0x00 } },
	{ 0xf193, 1, { 0x00 } },
	{ 0xf19d, 1, { 0x40 } },
	{ 0xf1bc, 2, { 0x36, 0x00 } },
	{ 0xf1cb, 2, { 0xa0, 0x01 } },
	{ 0xf204, 1, { 0x11 } },
	{ 0xf214, 1, { 0x00 } },
	{ 0xf40e, 3, { 0x0a, 0x40, 0x08 } },
	{ 0xf4c2, 1, { 0x00 } },
	{ 0xf506, 1, { 0x04 } },
	{ 0xf542, 1, { 0x02 } },
	{ 0xf55f, 1, { 0x0a } },
	{ 0xf561, 2, { 0x15, 0x20 } },
	{ 0xf5df, 2, { 0xfb, 0x00 } },
	{ 0xf5e3, 3, { 0x09, 0x01, 0x01 } },
	{ 0xf5f8, 1, { 0x01 } },
	{ 0xf5fd, 1, { 0x01 } },
	{ 0xf600, 8, { 0x05, 0x08, 0x0b, 0x0e, 0x11, 0x14, 0x17, 0x1f } },
	{ 0xf60e, 4, { 0x00, 0x04, 0x32, 0x10 } },
	{ 0xf62e, 1, { 0x64 } },
	{ 0xf707, 4, { 0xfc, 0x00, 0x37, 0x00 } },
	{ 0xf798, 1, { 0x0d } },
	{ 0xf80f, 3, { 0x40, 0x54, 0x5a } },
	{ 0xfb06, 1, { 0x03 } },
	{ 0xfd00, 1, { 0x01 } },
	{ 0xfd85, 1, { 0x00 } },
	{ 0xfd8a, 1, { 0x03 } },
	{ 0xfd9d, 2, { 0x00, 0x05 } },
	{ 0x0043, 1, { 0x00 } },
	{ 0x0046, 1, { 0x70 } },
	{ 0x005f, 2, { 0x00, 0x00 } },
	{ 0x0068, 1, { 0x20 } },
	{ 0x006a, 1, { 0x03 } },
	{ 0x0071, 2, { 0x05, 0x02 } },
	{ 0x0075, 5, { 0x8c, 0x8c, 0x8c, 0x8c, 0x01 } },
	{ 0x0082, 1, { 0x13 } },
	{ 0x0084, 6, { 0x0a, 0x33, 0xbe, 0x94, 0xc6, 0xa0 } },
	{ 0x0093, 4, { 0x00, 0x00, 0x00, 0x00 } },
	{ 0x009b, 2, { 0x3c, 0x28 } },
	{ 0x00a4, 1, { 0x5a } },
	{ 0x00b3, 2, { 0x02, 0x6a } },
	{ 0x00b6, 1, { 0x14 } },
	{ 0x00c0, 2, { 0x11, 0x00 } },
	{ 0x00c3, 2, { 0x01, 0x00 } },
	{ 0x00c7, 1, { 0x00 } },
	{ 0x00cc, 3, { 0x2c, 0x4f, 0x30 } },
	{ 0x0138, 1, { 0x01 } },
	{ 0x013b, 3, { 0x05, 0xa0, 0x8c } },
	{ 0x0140, 3, { 0x03, 0x06, 0x06 } },
	{ 0x0144, 10, { 0x03, 0x03, 0x02, 0x08, 0x50, 0x6e, 0x8c, 0x02, 0xbe, 0x00 } },
	{ 0x014f, 1, { 0x02 } },
	{ 0x015b, 2, { 0x0a, 0x03 } },
	{ 0x0161, 1, { 0xa0 } },
	{ 0x0164, 1, { 0x00 } },
	{ 0x0167, 4, { 0x50, 0x50, 0x47, 0x42 } },
	{ 0x0177, 1, { 0x03 } },
	{ 0x017a, 16, { 0x5b, 0x59, 0xf9, 0x59, 0x59, 0x8c, 0x8c, 0x8c, 0x6e, 0x8c, 0x50, 0x8c, 0x8c, 0xac, 0xc6, 0x33 } },
	{ 0x018b, 1, { 0x28 } },
	{ 0x018d, 4, { 0xbc, 0x00, 0x01, 0x00 } },
	{ 0x0192, 1, { 0x5e } },
	{ 0x0194, 1, { 0x09 } },
	{ 0x0197, 12, { 0x00, 0x00, 0x94, 0x6e, 0x02, 0xee, 0x32, 0x01, 0x46, 0x0e, 0x05, 0x06 } },
	{ 0x01b3, 2, { 0x02, 0x00 } },
	{ 0x01b7, 12, { 0x01, 0x07, 0x05, 0x03, 0x03, 0x01, 0x05, 0x27, 0x01, 0x3f, 0x02, 0x1f } },
	{ 0x01d4, 6, { 0x01, 0x05, 0x05, 0x05, 0x03, 0x05 } },
	{ 0xed1d, 4, { 0x06, 0x06, 0x06, 0x06 } },
	{ 0xf01f, 2, { 0x8c, 0x00 } },
	{ 0xf029, 2, { 0x8c, 0x00 } },
	{ 0xf077, 1, { 0x01 } },
	{ 0xf246, 3, { 0x01, 0x03, 0x01 } },
	{ 0xf24c, 5, { 0x88, 0x95, 0x9a, 0x90, 0x09 } },
	{ 0xf25a, 5, { 0x07, 0xe8, 0x03, 0xb0, 0x04 } },
	{ 0xf269, 1, { 0x07 } },
	{ 0xf270, 4, { 0x01, 0x02, 0x01, 0x02 } },
};

static const u8 it917x_sdram_clock[21][7] = {
    {0x65, 0x32, 0x80, 0x04, 0x4a, 0x48, 0x40},	//set  0 : 80MHz
	{0x65, 0x32, 0x78, 0x04, 0x4a, 0x48, 0x40},	//set  1 : 79MHz
	{0x65, 0x32, 0x70, 0x04, 0x4a, 0x48, 0x40},	//set  2 : 78MHz
	{0x65, 0x32, 0x68, 0x04, 0x4a, 0x48, 0x40},	//set  3 : 77MHz
	{0x65, 0x32, 0x60, 0x04, 0x4a, 0x48, 0x40},	//set  4 : 76MHz
	{0x65, 0x32, 0x58, 0x04, 0x4a, 0x48, 0x40},	//set  5 : 75MHz
	{0x65, 0x32, 0x50, 0x04, 0x4a, 0x48, 0x40},	//set  6 : 74MHz
	{0x65, 0x32, 0x48, 0x04, 0x4a, 0x48, 0x40},	//set  7 : 73MHz
	{0x65, 0x32, 0x40, 0x04, 0x4a, 0x48, 0x40},	//set  8 : 72MHz
	{0x65, 0x32, 0x38, 0x04, 0x4a, 0x48, 0x40},	//set  9 : 71MHz
    {0x65, 0x32, 0x30, 0x04, 0x4a, 0x48, 0x40},	//set 10 : 70MHz
    {0x65, 0x32, 0x28, 0x04, 0x4a, 0x48, 0x40},	//set 11 : 69MHz
    {0x65, 0x32, 0x20, 0x04, 0x4a, 0x48, 0x40},	//set 12 : 68MHz
    {0x65, 0x32, 0x18, 0x04, 0x4a, 0x48, 0x40},	//set 13 : 67MHz
	{0x65, 0x32, 0x10, 0x04, 0x4a, 0x48, 0x40},	//set 14 : 66MHz
	{0x65, 0x32, 0x08, 0x04, 0x4a, 0x48, 0x40},	//set 15 : 65MHz
    {0x65, 0x32, 0x00, 0x04, 0x4a, 0x48, 0x40},	//set 16 : 64MHz
    {0x65, 0x31, 0xf8, 0x04, 0x4a, 0x48, 0x40},	//set 17 : 63MHz
    {0x65, 0x31, 0xf0, 0x04, 0x4a, 0x48, 0x40},	//set 18 : 62MHz
    {0x65, 0x31, 0xe8, 0x04, 0x4a, 0x48, 0x40},	//set 19 : 61MHz
    {0x65, 0x31, 0xe0, 0x04, 0x4a, 0x48, 0x40},	//set 20 : 60MHz
};
static const u8 it917x_channel_map[70] = {
    0, 0, 0, 0, 0, 0, 0, 6, 3, 0,	        //ch0 ~ ch9
    5, 6, 1, 0, 16, 14, 14, 13, 13, 1,	    //ch10 ~ ch19
    0, 19, 18, 9, 18, 15, 14, 16, 13, 20,   //ch20 ~ ch29
	12, 1, 0, 18, 17, 16, 15, 14, 14, 14,   //ch30 ~ ch39
	13, 20, 6, 19, 1, 16, 15, 15, 14, 20,   //ch40 ~ ch49
	7, 20, 12, 18, 11, 16, 0, 1, 1, 20,     //ch50 ~ ch59
	20, 20, 17, 19, 17, 19, 16, 18, 15, 15, //ch60 ~ ch69
};

static const u8 it917x_nv[] = { 48, 32, 24, 16, 12, 8, 6, 4, 2 };

static int it917x_wait_calibration(struct it917x_frontend *it917x,
				 u16 reg, u8 *buf, u8 len)
{
#ifdef __linux__
	unsigned long deadline = jiffies + msecs_to_jiffies(100);
#else
	ULONGLONG deadline = GetTickCount64() + 100;
#endif
	int ret;

	for (;;) {
		ret = it917x_read_regs(it917x, IT917X_PROC_OFDM, reg, buf, len);
		if (ret)
			return ret;
		if (buf[0] || (len == 2 && buf[1]))
			return 0;
#ifdef __linux__
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		msleep(2);
#else
		if (GetTickCount64() >= deadline)
			return -ETIMEDOUT;
		Sleep(2);
#endif
	}
}

static int it917x_tuner_init(struct it917x_frontend *it917x)
{
	struct it917x_priv *priv = it917x->priv;
	u8 mode, nc, buf[2];
	int ret;

	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0xec86, &mode);
	if (ret)
		return ret;
	if (mode > 1)
		return -EINVAL;
	priv->clock_mode = mode;
	priv->fxtal_khz = mode ? 20480 : 2000;
	priv->fdiv = mode ? 18 : 3;
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0xed03, &nc);
	if (ret)
		return ret;
	if (nc >= ARRAY_SIZE(it917x_nv))
		return -EBADMSG;
	ret = it917x_wait_calibration(it917x, 0xed23, buf, 2);
	if (ret)
		return ret;

	priv->f_boundary = priv->fxtal_khz * ((buf[1] << 8) | buf[0]) /
		(priv->fdiv * it917x_nv[nc]);
	if (it917x->config.chip_id == 0x9175 && it917x->config.chip_version == 1) {
		msleep(50);
	} else {
		ret = it917x_wait_calibration(it917x, 0xec82, buf, 1);
		if (ret)
			return ret;
	}
	return it917x_write_reg(it917x, IT917X_PROC_OFDM, 0xed81, mode ? 6 : 16);
}

static int it917x_finish_setup(struct it917x_frontend *it917x)
{
	static const struct it917x_reg_val regs[] = {
		{ IT917X_PROC_OFDM, 0x0000, 1 },
		{ IT917X_PROC_LINK, 0xd827, 0 },
		{ IT917X_PROC_LINK, 0xd829, 0 },
	};
	static const struct it917x_reg_val ts_regs[] = {
		{ IT917X_PROC_LINK, 0xd8fd, 1 },
		{ IT917X_PROC_LINK, 0xd833, 1 },
		{ IT917X_PROC_LINK, 0xd830, 0 },
		{ IT917X_PROC_OFDM, 0xec57, 0 },
		{ IT917X_PROC_OFDM, 0xec58, 0 },
		/* キャプチャの選局時は ADC 2X。IF=0 の FCW は倍率によらず0。 */
		{ IT917X_PROC_OFDM, 0x0045, 1 },
		{ IT917X_PROC_LINK, 0xd831, 0 },
		{ IT917X_PROC_LINK, 0xd832, 0 },
		{ IT917X_PROC_LINK, 0xd809, 0 },
		{ IT917X_PROC_LINK, 0xd80b, 0 },
	};
	u8 zero[3] = { 0 };
	int ret, i;

	ret = it917x_write_regs(it917x, IT917X_PROC_OFDM, 0x0029, zero, 3);
	if (ret)
		return ret;
	for (i = 0; i < ARRAY_SIZE(it917x_script); i++) {
		ret = it917x_write_regs(it917x, IT917X_PROC_OFDM,
			it917x_script[i].reg, it917x_script[i].data,
			it917x_script[i].len);
		if (ret)
			return ret;
	}
	ret = it917x_write_reg_val(it917x, regs, ARRAY_SIZE(regs));
	if (ret)
		return ret;
	ret = it917x_tuner_init(it917x);
	if (ret)
		return ret;
	ret = it917x_write_reg_mask(it917x, IT917X_PROC_LINK, 0xf41a, 1, 1);
	if (ret)
		return ret;
	ret = it917x_write_reg_mask(it917x, IT917X_PROC_OFDM, 0xf985, 1, 1);
	if (ret)
		return ret;
	ret = it917x_write_reg_mask(it917x, IT917X_PROC_LINK, 0xd91c, 1, 1);
	if (ret)
		return ret;
	ret = it917x_write_reg(it917x, IT917X_PROC_OFDM, 0xcfff, 4);
	if (ret)
		return ret;
	return it917x_write_reg_val(it917x, ts_regs, ARRAY_SIZE(ts_regs));
}

int it917x_setup(struct it917x_frontend *it917x)
{
	int ret = 0;
	u8 buf[4];

	ret = it917x_read_chip_id(it917x, &it917x->config.chip_id, &it917x->config.chip_version);
	if (ret)
		return ret;
	if (it917x->config.chip_id != 0x9173 && it917x->config.chip_id != 0x9175)
		return -ENODEV;

	ret = it917x_read_xtal_freq(it917x, &it917x->config.xtal_hz, &it917x->config.adc_hz);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0x4bfb, 0);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0xcfff, it917x->config.clock_out ? 0x01 : 0x00);
	if (ret)
		return ret;

	ret = it917x_load_firmware(it917x, IT917X_FIRMWARE_FILENAME);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0xf103, 0x1a);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_OFDM, 0xec4c, 0xa8);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0xe00c, 0x00);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0xd864, 0x00);
	if (ret)
		return ret;

	ret = it917x_write_reg(it917x, IT917X_PROC_LINK, 0xf641, it917x->config.tuner_id);
	if (ret)
		return ret;

	ret = it917x_write_reg_mask(it917x, IT917X_PROC_OFDM, 0xf5ca, 0x01, 0x01);
	if (ret)
		return ret;

	ret = it917x_write_reg_mask(it917x, IT917X_PROC_OFDM, 0xf715, 0x01, 0x01);
	if (ret)
		return ret;

	ret = it917x_get_divider_xtal(it917x->config.xtal_hz, buf, sizeof(buf));
	if (ret)
		return ret;
	ret = it917x_write_regs(it917x, IT917X_PROC_OFDM, 0x0025, buf, 4);
	if (ret)
		return ret;

	ret = it917x_get_divider_adc(it917x->config.adc_hz, buf, sizeof(buf));
	if (ret)
		return ret;
	ret = it917x_write_regs(it917x, IT917X_PROC_OFDM, 0xf1cd, buf, 3);
	if (ret)
		return ret;

	return it917x_finish_setup(it917x);
}

static int it917x_set_bandwidth(struct it917x_frontend *it917x)
{
	static const u8 sfoe[2][24] = {
		{ 0x5e,0x45,0x5b,0x7f,0x56,0x6a,0x4d,0xc5,0x2c,0xf9,0x2b,0xa7,
		  0x29,0x3a,0x25,0x1b,0x17,0x01,0x16,0x53,0x15,0x16,0x12,0xfa },
		{ 0x5f,0x57,0x5c,0x89,0x57,0x65,0x4e,0xa8,0x2d,0x7c,0x2c,0x26,
		  0x29,0xb2,0x25,0x86,0x17,0x44,0x16,0x94,0x15,0x53,0x13,0x31 },
	};
	static const u32 coeff[2][8] = {
		{ 0x0335edd7,0x019af6eb,0x00cd81e2,0x00cd7b76,
		  0x00cd750a,0x0066bdbb,0x019af6eb,0x00cd7b76 },
		{ 0x032cb2cb,0x01965966,0x00cb330c,0x00cb2cb3,
		  0x00cb265a,0x00659659,0x01965966,0x00cb2cb3 },
	};
	u8 buf[36], multiplier;
	int i, ret, clock = it917x->config.adc_hz == 20480000;

	ret = it917x_write_regs(it917x, IT917X_PROC_OFDM, 0x011b,
		sfoe[clock], 24);
	if (ret)
		return ret;
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0x0045, &multiplier);
	if (ret)
		return ret;
	for (i = 0; i < 8; i++) {
		u32 value = coeff[clock][i] >> (multiplier == 1);
		buf[i * 4] = value >> 24;
		buf[i * 4 + 1] = value >> 16;
		buf[i * 4 + 2] = value >> 8;
		buf[i * 4 + 3] = value;
	}
	buf[32] = clock ? 0x85 : 0x7e;
	buf[33] = 2;
	buf[34] = clock ? 0x96 : 0x9b;
	buf[35] = 1;
	return it917x_write_regs(it917x, IT917X_PROC_OFDM, 0x0001, buf, sizeof(buf));
}

static unsigned int it917x_sdram_channel(u32 freq)
{
	if (freq < 400000)
		return freq <= 177143 ? 7 : min_t(u32, 13, (freq - 177143 + 2999) / 6000 + 7);
	return freq <= 473143 ? 14 : min_t(u32, 69, (freq - 473143 + 2999) / 6000 + 14);
}

int it917x_set_frequency(struct it917x_frontend *it917x, u32 freq, u32 bandwidth)
{
	struct it917x_priv *priv = it917x->priv;
	static const u32 boundaries[] = { 74000,111000,148000,222000,296000,445000 };
	static const u32 lna_bands[] = { 440000,484000,533000,587000,645000,710000,782000 };
	static const u16 clock_regs[] = { 0xfb25,0xfbb5,0xfbb6,0xfbb7,0xfbb9,0xfbb9,0xfbba };
	u8 nc = 0, lna = 0, ctrl, cal, pll;
	u32 mv, nv, code, cal_code;
	int ret, i, channel;

	/* PTX_SET_CHANNEL のCATV C13（111143kHz）から既存UHF範囲までを受け付ける。 */
	if (!priv || !priv->fxtal_khz || bandwidth != 6 || freq < 111143 || freq > 770000)
		return -EINVAL;
	ret = it917x_set_bandwidth(it917x);
	if (ret)
		return ret;
	ret = it917x_write_reg(it917x, IT917X_PROC_OFDM, 0x0040, 0);
	if (ret)
		return ret;
	ret = it917x_write_reg(it917x, IT917X_PROC_OFDM, 0x0047, 0);
	if (ret)
		return ret;
	ret = it917x_write_reg_mask(it917x, IT917X_PROC_OFDM, 0xf999, 0, 1);
	if (ret)
		return ret;
	ret = it917x_write_reg(it917x, IT917X_PROC_OFDM, 0x004b, freq <= 300000 ? 0 : 1);
	if (ret)
		return ret;
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0x01c6, &pll);
	if (ret)
		return ret;
	if (pll) {
		channel = it917x_sdram_channel(freq);
		for (i = 0; i < ARRAY_SIZE(clock_regs); i++) {
			ret = it917x_write_reg(it917x, IT917X_PROC_OFDM, clock_regs[i],
				it917x_sdram_clock[it917x_channel_map[channel]][i]);
			if (ret)
				return ret;
		}
	}
	while (nc < ARRAY_SIZE(boundaries) && freq > boundaries[nc])
		nc++;
	if (nc == 6 && freq > priv->f_boundary)
		nc++;
	if (nc == 7 && freq > 950000)
		nc++;
	nv = it917x_nv[nc];
	mv = (freq * nv * priv->fdiv + priv->fxtal_khz / 2) / priv->fxtal_khz;
	code = ((nc & 7) << 13) + mv;
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0xec4c, &ctrl);
	if (ret)
		return ret;
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0xed81, &cal);
	if (ret)
		return ret;
	if (cal > 64)
		return -EBADMSG;
	i = cal < 32 ? cal : 64 - cal;
	i = priv->clock_mode ? (i * nv) >> 1 : (i * nv * 9) >> 5;
	cal_code = ((nc & 7) << 13) + (cal < 32 ? mv + i : mv - i);
	while (lna < ARRAY_SIZE(lna_bands) && freq > lna_bands[lna])
		lna++;
	{
		const struct it917x_reg_val regs[] = {
			{ IT917X_PROC_OFDM, 0x0160, lna },
			{ IT917X_PROC_OFDM, 0xec56, 2 },
			{ IT917X_PROC_OFDM, 0xec4c, (ctrl & 0xe7) | (freq < 392000 ? 0 : 8) },
			{ IT917X_PROC_OFDM, 0xec4d, cal_code & 0xff },
			{ IT917X_PROC_OFDM, 0xec4e, cal_code >> 8 },
			{ IT917X_PROC_OFDM, 0x015e, code & 0xff },
			{ IT917X_PROC_OFDM, 0x015f, code >> 8 },
			{ IT917X_PROC_OFDM, 0x0000, 0 },
		};
		return it917x_write_reg_val(it917x, regs, ARRAY_SIZE(regs));
	}
}

/*
 * ioctl(PTX_GET_CNR, &val)で取得するvalは、TC90522のOFDM側レジスタ0x8b-0x8dの値
 * it917xでの取得値はCNRの整数部のため、CNRからvalへ以下のテーブルで変換する。
 */
static const u32 it917x_cnr_raw[256] = {
	0x000000, 0x000000, 0x9534ff, 0x5783f7, 0x3b8f3f, 0x2ae5c1, 0x1fe0f0, 0x182a40,
	0x12911f, 0x0e66fb, 0x0b3fa1, 0x08d42a, 0x06f42a, 0x057d7b, 0x045765, 0x036fa8,
	0x02b88e, 0x0227a3, 0x01b4da, 0x0159ec, 0x0111ed, 0x00d8fa, 0x00abfb, 0x008879,
	0x006c7e, 0x005676, 0x004520, 0x00377d, 0x002cbf, 0x002447, 0x001d95, 0x001846,
	0x00140c, 0x0010ab, 0x000df3, 0x000bc1, 0x0009f8, 0x000882, 0x00074e, 0x00064f,
	0x00057b, 0x0004c9, 0x000433, 0x0003b4, 0x000348, 0x0002eb, 0x00029b, 0x000256,
	0x00021a, 0x0001e6, 0x0001b8, 0x00018f, 0x00016c, 0x00014c, 0x000130, 0x000117,
	0x000101, 0x0000ed, 0x0000db, 0x0000ca, 0x0000bc, 0x0000ae, 0x0000a2, 0x000097,
	0x00008d, 0x000084, 0x00007b, 0x000074, 0x00006d, 0x000066, 0x000060, 0x00005a,
	0x000055, 0x000050, 0x00004c, 0x000048, 0x000044, 0x000041, 0x00003d, 0x00003a,
	0x000037, 0x000034, 0x000032, 0x000030, 0x00002d, 0x00002b, 0x000029, 0x000027,
	0x000026, 0x000024, 0x000022, 0x000021, 0x000020, 0x00001e, 0x00001d, 0x00001c,
	0x00001b, 0x00001a, 0x000019, 0x000018, 0x000017, 0x000016, 0x000015, 0x000014,
	0x000014, 0x000013, 0x000012, 0x000011, 0x000011, 0x000010, 0x000010, 0x00000f,
	0x00000f, 0x00000e, 0x00000e, 0x00000d, 0x00000d, 0x00000c, 0x00000c, 0x00000c,
	0x00000b, 0x00000b, 0x00000b, 0x00000a, 0x00000a, 0x00000a, 0x000009, 0x000009,
	0x000009, 0x000009, 0x000008, 0x000008, 0x000008, 0x000008, 0x000007, 0x000007,
	0x000007, 0x000007, 0x000007, 0x000006, 0x000006, 0x000006, 0x000006, 0x000006,
	0x000006, 0x000005, 0x000005, 0x000005, 0x000005, 0x000005, 0x000005, 0x000005,
	0x000005, 0x000004, 0x000004, 0x000004, 0x000004, 0x000004, 0x000004, 0x000004,
	0x000004, 0x000004, 0x000004, 0x000004, 0x000003, 0x000003, 0x000003, 0x000003,
	0x000003, 0x000003, 0x000003, 0x000003, 0x000003, 0x000003, 0x000003, 0x000003,
	0x000003, 0x000003, 0x000003, 0x000003, 0x000002, 0x000002, 0x000002, 0x000002,
	0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002,
	0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002,
	0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002, 0x000002,
	0x000001, 0x000001, 0x000001, 0x000001, 0x000001, 0x000001, 0x000001, 0x000001,
	0x000001, 0x000001, 0x000001, 0x000001, 0x000001, 0x000001, 0x000001, 0x000001,
	0x000001, 0x000001, 0x000001, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
	0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
	0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
	0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000, 0x000000,
};

int it917x_read_cnr_raw(struct it917x_frontend *it917x, u32 *value)
{
	u8 snr;
	int ret;

	if (!value || !it917x || !it917x->priv)
		return -EINVAL;
	*value = 0;
	ret = it917x_read_snr(it917x, &snr);
	if (ret)
		return ret;
	if (snr && !it917x_cnr_raw[snr])
		return -ERANGE;
	*value = it917x_cnr_raw[snr];
	return 0;
}

int it917x_read_snr(struct it917x_frontend *it917x, u8 *value)
{
	bool locked;
	int ret;

	if (!value || !it917x || !it917x->priv)
		return -EINVAL;
	*value = 0;
	ret = it917x_is_locked(it917x, &locked);
	if (ret || !locked)
		return ret;
	return it917x_read_reg(it917x, IT917X_PROC_OFDM, 0x01c9, value);
}

int it917x_is_locked(struct it917x_frontend *it917x, bool *locked)
{
	u8 tmcc, ts;
	int ret;

	if (!locked)
		return -EINVAL;
	*locked = false;
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0xf5a9, &tmcc);
	if (ret || !(tmcc & 1))
		return ret;
	/* LayerAのみの13セグ信号もあるため、LayerBではなく全体TSロックで判定する。 */
	ret = it917x_read_reg(it917x, IT917X_PROC_OFDM, 0xf999, &ts);
	if (!ret)
		*locked = !!(ts & 1);
	return ret;
}
