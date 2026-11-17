// isdbt6013_device.hpp

#pragma once

#include <cstdint>
#include <memory>
#include <atomic>
#include <string>
#include <mutex>
#include <condition_variable>

#include "device_definition_set.hpp"
#include "device_base.hpp"
#include "receiver_base.hpp"
#include "receiver_manager.hpp"
#include "ringbuffer.hpp"

#include "winusb_compat.h"

#include "i2c_comm.h"
#include "it930x.h"
#include "itedtv_bus.h"
#include "it917x.h"
#include <chrono>

namespace px4 {

#define ISDBT6013_DEVICE_TS_SYNC_COUNT	4
#define ISDBT6013_DEVICE_TS_SYNC_SIZE	(188 * ISDBT6013_DEVICE_TS_SYNC_COUNT)

struct Isdbt6013DeviceConfig final {
	Isdbt6013DeviceConfig()
		: usb{ 816, 816, 5, false },
		device{ 2048, 2000, true }
	{}
	struct {
		unsigned int xfer_packets;
		unsigned int urb_max_packets;
		unsigned int max_urbs;
		bool no_raw_io;
	} usb;
	struct {
		unsigned int receiver_max_packets;
		int psb_purge_timeout;
		bool discard_null_packets;
	} device;
};

class Isdbt6013Device final : public px4::DeviceBase {
public:
	Isdbt6013Device() = delete;
	Isdbt6013Device(const std::wstring &path, const px4::DeviceDefinition &device_def, std::uintptr_t index, px4::ReceiverManager &receiver_manager);
	~Isdbt6013Device();

	// cannot copy
	Isdbt6013Device(const Isdbt6013Device &) = delete;
	Isdbt6013Device& operator=(const Isdbt6013Device &) = delete;

	// cannot move
	Isdbt6013Device(Isdbt6013Device &&) = delete;
	Isdbt6013Device& operator=(Isdbt6013Device &&) = delete;

	int Init() override;
	void Term() override;
	void SetAvailability(bool available) override;
	px4::ReceiverBase* GetReceiver(int id) const override;

	bool HasCardReader() const noexcept override { return true; }
	int OpenCard() override;
	void CloseCard() override;
	int DetectCard(bool &detected) override;
	int ResetCard() override;
	int SetCardBaudrate(std::uint32_t baudrate) override;
	int IsCardDataReady(bool &ready) override;
	int ReadCardData(std::uint8_t *buf, std::uint8_t &len) override;
	int WriteCardData(const std::uint8_t *buf, std::uint8_t len) override;

private:
	friend struct Isdbt6013DeviceTest;
	struct StreamContext final {
		std::shared_ptr<px4::ReceiverBase::StreamBuffer> stream_buf[5];
		// 選局中の入力だけを止め、他入力とパイプ接続を維持する。
		std::mutex packet_lock[5];
		bool deliver[5] = {};
		// 共有端数に残った旧チャンネルのパケットだけを選局後に除外する。
		std::uint64_t generation[5] = {};
		std::uint64_t remain_generation[5] = {};
		/* 最初のコールバック前も未処理データなしの状態として初期化 */
		std::uint8_t remain_buf[ISDBT6013_DEVICE_TS_SYNC_SIZE] = {};
		std::size_t remain_len = 0;
	};

	class Isdbt6013Receiver final : public px4::ReceiverBase {
	public:
		Isdbt6013Receiver() = delete;
		Isdbt6013Receiver(Isdbt6013Device &parent, std::uintptr_t index);
		~Isdbt6013Receiver();

		// cannot copy
		Isdbt6013Receiver(const Isdbt6013Receiver &) = delete;
		Isdbt6013Receiver& operator=(const Isdbt6013Receiver &) = delete;

		// cannot move
		Isdbt6013Receiver(Isdbt6013Receiver &&) = delete;
		Isdbt6013Receiver& operator=(Isdbt6013Receiver &&) = delete;

		int Open() override;
		void Close() override;
		int CheckLock(bool &locked) override;
		int SetLnbVoltage(std::int32_t voltage) override;
		int SetCapture(bool capture) override;
		int ReadStat(px4::command::StatType type, std::int32_t &value) override;

	protected:
		int SetFrequency() override;
		int SetStreamId() override;

	private:
		friend struct Isdbt6013DeviceTest;
		Isdbt6013Device &parent_;
		std::uintptr_t index_;

		std::mutex lock_;
		std::atomic_bool open_;
		std::condition_variable close_cond_;
		it917x_frontend frontend_{};
		bool locked_ = false;
		std::chrono::steady_clock::time_point ready_at_{};
		bool streaming_ = false;
	};

	void LoadConfig();

	const i2c_comm_master& GetI2cMaster(int bus) const;

	int SetBackendPower(bool state);

	int StartCapture();
	int StopCapture();

	static void StreamProcess(StreamContext &ctx, int num, std::uint8_t **buf, std::size_t &len, const std::uint64_t generation[]);
	static int StreamHandler(void *context, void *buf, std::uint32_t len);

	static const struct Isdbt6013DeviceParam final {
		std::uint8_t i2c_addr;
		std::uint8_t i2c_bus;
		std::uint8_t port_number;
	} params_[5];

	Isdbt6013DeviceConfig config_;
	std::recursive_mutex lock_;
	std::atomic_bool available_;
	std::atomic_bool init_;
	unsigned int open_count_;
	// 基板電源を共有するカード利用はチューナー参照とは別に保持する。
	bool card_open_ = false;
	unsigned int streaming_count_;
	int receiver_num_;
	std::unique_ptr<Isdbt6013Receiver> receivers_[5];
	it930x_bridge it930x_{};
	StreamContext stream_ctx_;
};

} // namespace px4
