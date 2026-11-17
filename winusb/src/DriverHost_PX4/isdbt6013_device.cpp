// isdbt6013_device.cpp

#include "isdbt6013_device.hpp"

#include <cassert>
#include <thread>
#include <algorithm>

#include "type.hpp"
#include "command.hpp"
#include "misc_win.h"
#include "ts_sync.h"

namespace px4 {

const Isdbt6013Device::Isdbt6013DeviceParam Isdbt6013Device::params_[5] = {
	{ 0x1c, 3, 0 }, { 0x1c, 1, 1 }, { 0x1e, 1, 2 },
	{ 0x1e, 3, 3 }, { 0x1f, 3, 4 }
};

Isdbt6013Device::Isdbt6013Device(const std::wstring &path, const px4::DeviceDefinition &device_def, std::uintptr_t index, px4::ReceiverManager &receiver_manager)
	: DeviceBase(path, device_def, index, receiver_manager),
	config_(),
	lock_(),
	available_(true),
	init_(false),
	open_count_(0),
	streaming_count_(0)
{
	if (usb_dev_.descriptor.idVendor != 0x0511)
		throw DeviceError("px4::Isdbt6013Device::Isdbt6013Device: unsupported device. (unknown vendor id)");

	receiver_num_ = 5;

	if (usb_dev_.descriptor.idProduct != 0x024d)
		throw DeviceError("unsupported ISDBT6013 product id");

	LoadConfig();
	// 188倍の転送長が桁あふれするとUSBバッファと設定値が一致しなくなる。
	if (!config_.usb.xfer_packets || config_.usb.xfer_packets > UINT_MAX / 188 ||
		!config_.usb.urb_max_packets || config_.usb.urb_max_packets > UINT_MAX / 188 ||
		!config_.usb.max_urbs || !config_.device.receiver_max_packets ||
		config_.device.receiver_max_packets > UINT_MAX / 188 || config_.device.psb_purge_timeout < 0)
		throw DeviceError("invalid ISDBT6013 buffer configuration");

	memset(&it930x_, 0, sizeof(it930x_));
}

Isdbt6013Device::~Isdbt6013Device()
{
	Term();
}

void Isdbt6013Device::LoadConfig()
{
	auto &configs = device_def_.configs;

	if (configs.Exists(L"XferPackets"))
		config_.usb.xfer_packets = px4::util::wtoui(configs.Get(L"XferPackets"));

	if (configs.Exists(L"UrbMaxPackets"))
		config_.usb.urb_max_packets = px4::util::wtoui(configs.Get(L"UrbMaxPackets"));

	if (configs.Exists(L"MaxUrbs"))
		config_.usb.max_urbs = px4::util::wtoui(configs.Get(L"MaxUrbs"));

	if (configs.Exists(L"NoRawIo"))
		config_.usb.no_raw_io = px4::util::wtob(configs.Get(L"NoRawIo"));

	if (configs.Exists(L"ReceiverMaxPackets"))
		config_.device.receiver_max_packets = px4::util::wtoui(configs.Get(L"ReceiverMaxPackets"));

	if (configs.Exists(L"PsbPurgeTimeout"))
		config_.device.psb_purge_timeout = px4::util::wtoi(configs.Get(L"PsbPurgeTimeout"));

	if (configs.Exists(L"DiscardNullPackets"))
		config_.device.discard_null_packets = px4::util::wtob(configs.Get(L"DiscardNullPackets"));

	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: xfer_packets: %u\n", config_.usb.xfer_packets);
	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: urb_max_packets: %u\n", config_.usb.urb_max_packets);
	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: max_urbs: %u\n", config_.usb.max_urbs);
	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: no_raw_io: %s\n", (config_.usb.no_raw_io) ? "true" : "false");
	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: receiver_max_packets: %u\n", config_.device.receiver_max_packets);
	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: psb_purge_timeout: %i\n", config_.device.psb_purge_timeout);
	dev_dbg(&dev_, "px4::Isdbt6013Device::LoadConfig: discard_null_packets: %s\n", (config_.device.discard_null_packets) ? "true" : "false");

	return;
}

int Isdbt6013Device::Init()
{
	dev_dbg(&dev_, "px4::Isdbt6013Device::Init: init_: %s\n", (init_) ? "true" : "false");

	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (init_)
		return -EALREADY;

	int ret = 0;
	itedtv_bus &bus = it930x_.bus;

	bus.dev = &dev_;
	bus.type = ITEDTV_BUS_USB;
	bus.usb.dev = &usb_dev_;
	bus.usb.ctrl_timeout = 3000;

	it930x_.dev = &dev_;
	it930x_.config.xfer_size = 188 * config_.usb.xfer_packets;
	it930x_.config.i2c_speed = 0x07;

	ret = itedtv_bus_init(&bus);
	if (ret)
		goto fail_bus;

	ret = it930x_init(&it930x_);
	if (ret)
		goto fail_bridge;

	for (int i = 0; i < receiver_num_; i++) {
		it930x_stream_input &input = it930x_.config.input[i];

		input.enable = true;
		input.is_parallel = false;
		input.port_number = params_[i].port_number;
		input.slave_number = i;
		input.i2c_bus = params_[i].i2c_bus;
		input.i2c_addr = params_[i].i2c_addr;
		input.packet_len = 188;
		input.sync_byte = ((i + 1) << 4) | 0x07;

		receivers_[i].reset(new Isdbt6013Receiver(*this, i));
		stream_ctx_.stream_buf[i] = receivers_[i]->GetStreamBuffer();
	}

	ret = it930x_raise(&it930x_);
	if (ret)
		goto fail_device;

	ret = it930x_load_firmware(&it930x_, "it930x-firmware.bin");
	if (ret)
		goto fail_device;

	ret = it930x_init_warm(&it930x_);
	if (ret)
		goto fail_device;

	/* GPIO */
	ret = it930x_set_gpio_mode(&it930x_, 7, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail_device;

	ret = it930x_write_gpio(&it930x_, 7, true);
	if (ret)
		goto fail_device;

	ret = it930x_set_gpio_mode(&it930x_, 2, IT930X_GPIO_OUT, true);
	if (ret)
		goto fail_device;

	ret = it930x_write_gpio(&it930x_, 2, false);
	if (ret)
		goto fail_device;

	if (config_.device.discard_null_packets) {
		it930x_pid_filter filter;

		filter.block = true;
		filter.num = 1;
		filter.pid[0] = 0x1fff;

		for (int i = 0; i < receiver_num_; i++) {
			ret = it930x_set_pid_filter(&it930x_, i, &filter);
			if (ret)
				goto fail_device;
		}
	}

	init_ = true;

	for (auto it = device_def_.receivers.cbegin(); it != device_def_.receivers.cend(); ++it) {
		px4::command::ReceiverInfo ri;

		if (it->index < 0 || it->index >= receiver_num_)
			continue;

		wcscpy_s(ri.device_name, device_def_.name.c_str());
		ri.device_guid = device_def_.guid;
		wcscpy_s(ri.receiver_name, it->name.c_str());
		ri.receiver_guid = it->guid;
		ri.systems = it->systems;
		ri.index = it->index;
		ri.data_id = 0;

		receiver_manager_.Register(ri, receivers_[it->index].get());
	}

	return 0;

fail_device:
	for (int i = 0; i < receiver_num_; i++) {
		receivers_[i].reset();
	}

	for (auto &buffer : stream_ctx_.stream_buf)
		buffer.reset();
	it930x_term(&it930x_);

fail_bridge:
	itedtv_bus_term(&bus);

fail_bus:
	return ret;
}

void Isdbt6013Device::Term()
{
	dev_dbg(&dev_, "px4::Isdbt6013Device::Term: init_: %s\n", (init_) ? "true" : "false");

	std::unique_lock<std::recursive_mutex> lock(lock_);

	if (!init_)
		return;

	CloseCard();
	init_ = false;
	// SearchAndOpenは一覧ロックからOpenへ入るため、その逆順で取得しない。
	lock.unlock();
	for (int i = 0; i < receiver_num_; i++)
		receiver_manager_.Unregister(receivers_[i].get());

	for (int i = 0; i < receiver_num_; i++) {
		receivers_[i].reset();
	}

	lock.lock();

	for (auto &buffer : stream_ctx_.stream_buf)
		buffer.reset();
	it930x_term(&it930x_);
	itedtv_bus_term(&it930x_.bus);

	return;
}

void Isdbt6013Device::SetAvailability(bool available)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	// 抜去後のカード要求が古い電源参照を再利用しないようにする。
	if (!available)
		card_open_ = false;
	available_ = available;
	if (!available) {
		for (int i = 0; i < receiver_num_; ++i) {
			std::lock_guard<std::mutex> packet_lock(stream_ctx_.packet_lock[i]);
			stream_ctx_.deliver[i] = false;
			if (stream_ctx_.stream_buf[i])
				stream_ctx_.stream_buf[i]->StopRequest();
		}
	}
}

ReceiverBase* Isdbt6013Device::GetReceiver(int id) const
{
	if (id < 0 || id >= receiver_num_)
		throw std::out_of_range("receiver id out of range");

	return receivers_[id].get();
}

int Isdbt6013Device::OpenCard()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!available_ || !init_ || !HasCardReader())
		return -ENODEV;
	if (card_open_)
		return -EBUSY;

	/* チューナー未使用時だけ基板の電源を投入し、視聴中の電源状態には触れない */
	if (!open_count_) {
		int ret = SetBackendPower(true);
		if (ret) {
			// GPIO操作の途中で失敗しても、未使用基板の電源を残さない。
			SetBackendPower(false);
			return ret;
		}
	}

	int ret = it930x_bcas_init(&it930x_);
	if (ret) {
		if (!open_count_)
			SetBackendPower(false);
		return ret;
	}

	card_open_ = true;
	return 0;
}

void Isdbt6013Device::CloseCard()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!card_open_)
		return;

	card_open_ = false;
	if (!open_count_)
		SetBackendPower(false);
}

int Isdbt6013Device::DetectCard(bool &detected)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	return available_ && init_ && HasCardReader() ?
		it930x_bcas_detect_card(&it930x_, &detected) : -ENODEV;
}

int Isdbt6013Device::ResetCard()
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	return available_ && init_ && card_open_ ? it930x_bcas_reset_card(&it930x_) : -ENODEV;
}

int Isdbt6013Device::SetCardBaudrate(std::uint32_t baudrate)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	return available_ && init_ && card_open_ ? it930x_bcas_set_baudrate(&it930x_, baudrate) : -ENODEV;
}

int Isdbt6013Device::IsCardDataReady(bool &ready)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	return available_ && init_ && card_open_ ? it930x_bcas_check_ready(&it930x_, &ready) : -ENODEV;
}

int Isdbt6013Device::ReadCardData(std::uint8_t *buf, std::uint8_t &len)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	return available_ && init_ && card_open_ ? it930x_bcas_get_data(&it930x_, buf, &len) : -ENODEV;
}

int Isdbt6013Device::WriteCardData(const std::uint8_t *buf, std::uint8_t len)
{
	std::lock_guard<std::recursive_mutex> lock(lock_);
	return available_ && init_ && card_open_ ? it930x_bcas_send_data(&it930x_, buf, len) : -ENODEV;
}

const i2c_comm_master& Isdbt6013Device::GetI2cMaster(int bus) const
{
	if (bus < 1 || bus > 3)
		throw std::out_of_range("bus number out of range");

	return it930x_.i2c_master[bus - 1];
}

int Isdbt6013Device::SetBackendPower(bool state)
{
	dev_dbg(&dev_, "px4::Isdbt6013Device::SetBackendPower: state: %s\n", (state) ? "true" : "false");

	int ret = 0;
	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!state && !available_)
		return 0;

	if (state) {
		ret = it930x_write_gpio(&it930x_, 7, false);
		if (ret)
			return ret;

		Sleep(80);

		ret = it930x_write_gpio(&it930x_, 2, true);
		if (ret)
			return ret;

		Sleep(20);
	} else {
		it930x_write_gpio(&it930x_, 2, false);
		it930x_write_gpio(&it930x_, 7, true);
	}

	return 0;
}

int Isdbt6013Device::StartCapture()
{
	dev_dbg(&dev_, "px4::Isdbt6013Device::StartCapture\n");

	int ret = 0;
	std::lock_guard<std::recursive_mutex> lock(lock_);
	if (!available_ || !init_)
		return -ENODEV;

	if (!streaming_count_) {
		ret = it930x_purge_psb(&it930x_, config_.device.psb_purge_timeout);
		if (ret) {
			dev_err(&dev_, "px4::Isdbt6013Device::StartCapture: it930x_purge_psb() failed. (ret: %d)\n", ret);
			return ret;
		}

		it930x_.bus.usb.streaming.urb_buffer_size = 188 * config_.usb.urb_max_packets;
		it930x_.bus.usb.streaming.urb_num = config_.usb.max_urbs;
		it930x_.bus.usb.streaming.no_dma = true;
		it930x_.bus.usb.streaming.no_raw_io = config_.usb.no_raw_io;

		stream_ctx_.remain_len = 0;

		ret = itedtv_bus_start_streaming(&it930x_.bus, StreamHandler, this);
		if (ret) {
			dev_err(&dev_, "px4::Isdbt6013Device::StartCapture: itedtv_bus_start_streaming() failed. (ret: %d)\n", ret);
			return ret;
		}
	}

	streaming_count_++;
	dev_dbg(&dev_, "px4::Isdbt6013Device::StartCapture: streaming_count_: %u\n", streaming_count_);

	return 0;
}

int Isdbt6013Device::StopCapture()
{
	dev_dbg(&dev_, "px4::Isdbt6013Device::StopCapture\n");

	std::lock_guard<std::recursive_mutex> lock(lock_);

	if (!streaming_count_)
		return -EALREADY;

	streaming_count_--;
	if (!streaming_count_) {
		dev_dbg(&dev_, "px4::Isdbt6013Device::StopCapture: stopping...\n");
		itedtv_bus_stop_streaming(&it930x_.bus);
	} else {
		dev_dbg(&dev_, "px4::Isdbt6013Device::StopCapture: streaming_count_: %u\n", streaming_count_);
	}

	return 0;
}

void Isdbt6013Device::StreamProcess(StreamContext &ctx, int num, std::uint8_t **buf, std::size_t &len, const std::uint64_t generation[])
{
	std::uint8_t *p = *buf;
	std::size_t remain = len;

	while (remain) {
		std::size_t i;
		bool sync_remain = false;

		for (i = 0; i < ISDBT6013_DEVICE_TS_SYNC_COUNT; i++) {
			if (((i + 1) * 188) <= remain) {
				if (!px4_ts_has_tagged_sync(p[i * 188]))
					break;
			} else {
				sync_remain = true;
				break;
			}
		}

		if (sync_remain)
			break;

		if (i < ISDBT6013_DEVICE_TS_SYNC_COUNT) {
			p++;
			remain--;
			continue;
		}

		while (remain >= 188 && px4_ts_has_tagged_sync(p[0])) {
			u8 id = (p[0] & 0x70) >> 4;

			if (id && id <= num) {
				std::size_t pkt_len = 188;

				p[0] = 0x47;
				std::lock_guard<std::mutex> packet_lock(ctx.packet_lock[id - 1]);
				if (ctx.deliver[id - 1] && ctx.generation[id - 1] == generation[id - 1])
					ctx.stream_buf[id - 1]->Write(p, pkt_len);
			}

			p += 188;
			remain -= 188;
		}
	}

	for (int i = 0; i < num; i++) {
		std::lock_guard<std::mutex> packet_lock(ctx.packet_lock[i]);
		ctx.stream_buf[i]->NotifyWrite();
	}

	*buf = p;
	len = remain;

	return;
}


int Isdbt6013Device::StreamHandler(void *context, void *buf, std::uint32_t len)
{
	auto &obj = *static_cast<Isdbt6013Device *>(context);
	auto &ctx = obj.stream_ctx_;
	auto *p = static_cast<std::uint8_t *>(buf);
	std::size_t remain = len;
	std::uint64_t generation[5];
	for (int i = 0; i < 5; ++i) {
		std::lock_guard<std::mutex> packet_lock(ctx.packet_lock[i]);
		generation[i] = ctx.generation[i];
	}

	// 同期探索で残った端数も順序を保つ。取り込んだ入力は二重に結合しない。
	while (ctx.remain_len && remain) {
		auto *pending = ctx.remain_buf;
		std::size_t copy = (std::min)(sizeof(ctx.remain_buf) - ctx.remain_len, remain);
		memcpy(pending + ctx.remain_len, p, copy);
		ctx.remain_len += copy;
		p += copy;
		remain -= copy;
		if (ctx.remain_len < sizeof(ctx.remain_buf))
			return 0;
		StreamProcess(ctx, 5, &pending, ctx.remain_len, ctx.remain_generation);
		memmove(ctx.remain_buf, pending, ctx.remain_len);
	}
	StreamProcess(ctx, 5, &p, remain, generation);
	if (remain) {
		memcpy(ctx.remain_buf, p, remain);
		ctx.remain_len = remain;
		memcpy(ctx.remain_generation, generation, sizeof(generation));
	}
	return 0;
}

Isdbt6013Device::Isdbt6013Receiver::Isdbt6013Receiver(Isdbt6013Device &parent, std::uintptr_t index)
	: ReceiverBase(0), parent_(parent), index_(index), open_(false)
{
	frontend_.dev = &parent_.dev_;
	frontend_.i2c = &parent_.GetI2cMaster(parent_.params_[index].i2c_bus);
	frontend_.i2c_addr = parent_.params_[index].i2c_addr;
}

Isdbt6013Device::Isdbt6013Receiver::~Isdbt6013Receiver()
{
	std::unique_lock<std::mutex> lock(lock_);
	// パイプが保持する受信機を先に破棄すると参照が残るため、既存機種と同じ寿命を保つ。
	close_cond_.wait(lock, [this] { return !open_; });
}

int Isdbt6013Device::Isdbt6013Receiver::Open()
{
	std::lock_guard<std::mutex> lock(lock_);
	std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);
	if (!parent_.available_ || !parent_.init_)
		return -ENODEV;
	if (open_)
		return -EALREADY;
	int ret = 0;
	if (!parent_.open_count_ && !parent_.card_open_) {
		ret = parent_.SetBackendPower(true);
		if (ret)
			goto fail;
	}
	ret = it917x_init(&frontend_);
	if (ret)
		goto fail;
	ret = it917x_setup(&frontend_);
	if (ret) {
		it917x_term(&frontend_);
		goto fail;
	}
	locked_ = false;
	ready_at_ = {};
	parent_.open_count_++;
	open_ = true;
	return 0;
fail:
	if (!parent_.open_count_ && !parent_.card_open_)
		parent_.SetBackendPower(false);
	return ret;
}

void Isdbt6013Device::Isdbt6013Receiver::Close()
{
	std::unique_lock<std::mutex> lock(lock_);
	if (!open_)
		return;
	std::lock_guard<std::recursive_mutex> dev_lock(parent_.lock_);
	if (streaming_) {
		parent_.StopCapture();
		streaming_ = false;
	}
	{
		std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
		parent_.stream_ctx_.deliver[index_] = false;
		stream_buf_->Stop();
	}
	it917x_term(&frontend_);
	locked_ = false;
	ready_at_ = {};
	open_ = false;
	assert(parent_.open_count_);
	if (!--parent_.open_count_ && !parent_.card_open_)
		parent_.SetBackendPower(false);
	close_cond_.notify_all();
}

int Isdbt6013Device::Isdbt6013Receiver::SetFrequency()
{
	std::lock_guard<std::mutex> lock(lock_);
	locked_ = false;
	ready_at_ = {};
	{
		std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
		parent_.stream_ctx_.deliver[index_] = false;
		++parent_.stream_ctx_.generation[index_];
	}
	if (!parent_.available_ || !parent_.init_)
		return -ENODEV;
	if (!open_ || params_.system != px4::SystemType::ISDB_T)
		return -EINVAL;
	// 他入力のUSB受信は継続し、この入力の旧チャンネルだけを整定完了まで止める。
	if (streaming_ && !stream_buf_->Purge())
		return -ETIMEDOUT;
	return it917x_set_frequency(&frontend_, params_.freq,
		params_.bandwidth ? params_.bandwidth : 6);
}

int Isdbt6013Device::Isdbt6013Receiver::CheckLock(bool &locked)
{
	std::lock_guard<std::mutex> lock(lock_);
	locked = false;
	if (!parent_.available_ || !parent_.init_) {
		locked_ = false;
		ready_at_ = {};
		return -ECANCELED;
	}
	if (!open_)
		return -EINVAL;
	bool hardware_locked = false;
	int ret = it917x_is_locked(&frontend_, &hardware_locked);
	if (ret || !hardware_locked) {
		locked_ = false;
		ready_at_ = {};
		std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
		parent_.stream_ctx_.deliver[index_] = false;
		return ret;
	}
	if (!locked_) {
		// ロック直後のTEIを避けるため、Linux版の実測値を受信機単位で確保する。
		ready_at_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(350);
	}
	std::this_thread::sleep_until(ready_at_);
	if (!parent_.available_ || !parent_.init_) {
		locked_ = false;
		ready_at_ = {};
		return -ECANCELED;
	}
	if (!locked_ && streaming_) {
		if (!stream_buf_->Purge())
			return -ETIMEDOUT;
		std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
		if (!parent_.available_ || !parent_.init_)
			return -ECANCELED;
		parent_.stream_ctx_.deliver[index_] = true;
	}
	locked_ = locked = true;
	return 0;
}

int Isdbt6013Device::Isdbt6013Receiver::SetCapture(bool capture)
{
	std::lock_guard<std::mutex> lock(lock_);
	if (!open_)
		return -EINVAL;
	if (capture == streaming_)
		return -EALREADY;
	if (capture) {
		if (!parent_.available_ || !parent_.init_)
			return -ENODEV;
		// BonDriverはOpenTunerで選局前に受信を開始する。USBだけを開始し、
		// 未選局の入力はCheckLockの整定完了まで配信しない。
		std::size_t size = 188 * parent_.config_.device.receiver_max_packets;
		{
			std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
			if (!parent_.available_ || !parent_.init_)
				return -ENODEV;
			if (!stream_buf_->Alloc(size))
				return -ENOMEM;
			stream_buf_->SetThresholdSize(size / 10);
			stream_buf_->Start();
			parent_.stream_ctx_.deliver[index_] = locked_;
		}
		int ret = parent_.StartCapture();
		if (ret) {
			std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
			parent_.stream_ctx_.deliver[index_] = false;
			stream_buf_->Stop();
			return ret;
		}
	} else {
		{
			std::lock_guard<std::mutex> packet_lock(parent_.stream_ctx_.packet_lock[index_]);
			parent_.stream_ctx_.deliver[index_] = false;
		}
		int ret = parent_.StopCapture();
		if (ret)
			return ret;
		stream_buf_->Stop();
	}
	streaming_ = capture;
	return 0;
}

int Isdbt6013Device::Isdbt6013Receiver::SetStreamId()
{
	return params_.system == px4::SystemType::ISDB_T ? 0 : -EINVAL;
}

int Isdbt6013Device::Isdbt6013Receiver::SetLnbVoltage(std::int32_t voltage)
{
	return voltage ? -EINVAL : 0;
}

int Isdbt6013Device::Isdbt6013Receiver::ReadStat(px4::command::StatType type, std::int32_t &value)
{
	std::lock_guard<std::mutex> lock(lock_);
	value = 0;
	if (!parent_.available_ || !parent_.init_)
		return -ENODEV;
	if (!open_)
		return -EINVAL;
	if (type != px4::command::StatType::CNR)
		return -ENOSYS;
	u8 snr = 0;
	int ret = it917x_read_snr(&frontend_, &snr);
	// 公開パイプのCNRは1/1000 dB。Linux PTX用の互換生値は使用しない。
	if (!ret)
		value = static_cast<std::int32_t>(snr) * 1000;
	return ret;
}

} // namespace px4
