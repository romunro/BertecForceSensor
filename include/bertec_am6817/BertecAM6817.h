#pragma once

#include <array>
#include <atomic>
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
};

/**
 * @brief Direct POSIX serial driver for the Bertec AM6817-E digital amplifier.
 *
 * Communicates over /dev/ttyUSB* at 1,500,000 baud 8N1 using HDLC-style
 * 0x7E framing with 0x7D (XOR 0x20) byte-stuffing.
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

  DeviceInfo getInfo();
  CalibrationData readCalibration();

  void startStream();
  void stopStream();

  /** Blocking read of one 1 kHz frame (call after startStream()). */
  Sample readSample();

  /** Blocking tare over n_samples frames (stops stream if needed, or call before startAsync). */
  void tareBlocking(std::size_t n_samples = 200);

  /** Start non-blocking background reader thread (for real-time loops like mc_rtc). */
  void startAsync();
  void stopAsync();
  bool isStreaming() const noexcept { return running_.load(); }

  /** Trigger a non-blocking software tare over the next n_samples in the background thread. */
  void requestTare(std::size_t n_samples = 200);
  bool isTaring() const noexcept { return tare_remaining_.load() > 0; }

  /** Thread-safe snapshot of the most recent sample. */
  Sample latestSample() const;

  const CalibrationData & calibration() const noexcept { return calibration_; }
  const DeviceInfo & info() const noexcept { return info_; }

  /** Read FTDI latency_timer in ms from sysfs, or -1 if unavailable. */
  int latencyTimerMs() const;

private:
  std::vector<uint8_t> readExact(std::size_t n, int timeout_ms);
  std::array<double, 6> convertRaw(const std::array<uint16_t, 8> & raw) const;
  void readerLoop();

  std::string port_;
  int timeout_ms_;
  int fd_ = -1;
  std::vector<uint8_t> rx_buf_;

  DeviceInfo info_{};
  CalibrationData calibration_{};

  mutable std::mutex data_mutex_;
  std::array<double, 8> baseline_{32768.0, 32768.0, 32768.0, 32768.0,
                                  32768.0, 32768.0, 32768.0, 32768.0};
  Sample latest_{};
  bool has_prev_seq_ = false;
  uint8_t prev_seq_ = 0;

  std::atomic<bool> running_{false};
  std::thread reader_thread_;

  std::atomic<std::size_t> tare_remaining_{0};
  std::size_t tare_total_ = 0;
  std::array<double, 8> tare_accum_{};
};

} // namespace bertec
