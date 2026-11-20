#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace bertec
{

struct DeviceInfo
{
  uint16_t firmware_raw = 0;
  uint16_t eeprom_size = 0;
  uint16_t sample_rate_hz = 1000;
  uint8_t num_channels = 8;
  std::string serial;
};

struct CalibrationData
{
  std::string serial;
  std::string model;
  std::vector<double> dimensions_mm;
  std::vector<double> ranges;
  /** 6x8 matrix: rows = [Fx, Fy, Fz, Mx, My, Mz], cols = 8 raw ADC channels. */
  std::array<std::array<double, 8>, 6> matrix{};
  bool valid = false;
};

struct Sample
{
  uint8_t seq = 0;
  std::array<uint16_t, 8> raw{};
  /** Calibrated wrench: [Fx, Fy, Fz (N), Mx, My, Mz (Nm)]. */
  std::array<double, 6> wrench{};
  uint64_t sample_count = 0;
  uint64_t dropped_frames = 0;
  uint64_t corrupted_frames = 0;
  uint64_t reconnect_count = 0;
  std::chrono::steady_clock::time_point timestamp{};
};

/**
 * @brief Direct POSIX serial driver for the Bertec AM6817-E digital amplifier.
 *
 * Communicates over /dev/ttyUSB* at 1,500,000 baud 8N1 using HDLC-style
 * 0x7E framing with 0x7D (XOR 0x20) byte-stuffing. Supports automatic
 * hot-plug reconnection even if Linux re-enumerates the ttyUSB node.
 */
class BertecAM6817
{
public:
  explicit BertecAM6817(std::string port = "/dev/ttyUSB0", int timeout_ms = 500);
  ~BertecAM6817();

  BertecAM6817(const BertecAM6817 &) = delete;
  BertecAM6817 & operator=(const BertecAM6817 &) = delete;

  void open();
  void close();
  bool isOpen() const noexcept { return fd_ >= 0; }

  /** Currently active resolved device node (e.g., /dev/ttyUSB0 or /dev/ttyUSB1). */
  std::string activePort() const;

  DeviceInfo getInfo();
  CalibrationData readCalibration();

  void startStream();
  void stopStream();

  /** Blocking read of one 1 kHz frame (call after startStream()). */
  Sample readSample();

  /** Blocking tare over n_samples frames (stops stream if needed, or call before startAsync). */
  void tareBlocking(std::size_t n_samples = 200);

  /**
   * @brief Start non-blocking background reader thread with automatic hot-plug reconnect.
   * @param initial_tare_samples If the device was not yet connected when startAsync() is called,
   *        number of samples to tare automatically once the first connection succeeds.
   */
  void startAsync(std::size_t initial_tare_samples = 0);
  void stopAsync();
  bool isStreaming() const noexcept { return running_.load(); }
  bool isStreamHealthy() const noexcept { return stream_healthy_.load(); }

  /** Trigger a non-blocking software tare over the next n_samples in the background thread. */
  void requestTare(std::size_t n_samples = 200);
  bool isTaring() const noexcept;

  /**
   * @brief Configure corruption sanity guard multiplier relative to EEPROM [Ranges].
   * Frames exceeding max_range_multiplier * Range (due to UART bit flips) are rejected.
   * Set <= 0.0 to disable. Default: 3.0.
   */
  void setMaxRangeMultiplier(double mult) noexcept { max_range_multiplier_ = mult; }

  /** Thread-safe snapshot of the most recent sample. */
  Sample latestSample() const;

  const CalibrationData & calibration() const noexcept { return calibration_; }
  const DeviceInfo & info() const noexcept { return info_; }

  /** Read FTDI latency_timer in ms from sysfs, or -1 if unavailable. */
  int latencyTimerMs() const;

private:
  std::string resolveDevicePath() const;
  void closeFdOnly() noexcept;
  bool tryReconnect();
  std::vector<uint8_t> readExact(std::size_t n, int timeout_ms);
  bool tryPopFrame(std::uint8_t & seq_out, std::array<uint16_t, 8> & raw_out);
  void drainAvailableNonBlocking();
  std::array<double, 6> convertRawUnlocked(const std::array<uint16_t, 8> & raw) const;
  bool isWrenchWithinSanityLimits(const std::array<double, 6> & w) const noexcept;
  void readerLoop();

  std::string port_;
  std::string active_port_;
  int timeout_ms_;
  int fd_ = -1;
  std::vector<uint8_t> rx_buf_;

  DeviceInfo info_{};
  CalibrationData calibration_{};
  double max_range_multiplier_ = 3.0;

  mutable std::mutex data_mutex_;
  std::array<double, 8> baseline_{32768.0, 32768.0, 32768.0, 32768.0,
                                  32768.0, 32768.0, 32768.0, 32768.0};
  bool has_baseline_ = false;
  std::size_t initial_tare_samples_ = 0;
  Sample latest_{};
  bool has_prev_seq_ = false;
  uint8_t prev_seq_ = 0;

  std::atomic<bool> running_{false};
  std::atomic<bool> stream_healthy_{false};
  std::thread reader_thread_;

  std::size_t tare_remaining_ = 0;
  std::size_t tare_total_ = 0;
  std::array<double, 8> tare_accum_{};
};

} // namespace bertec
