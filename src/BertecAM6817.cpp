#include <bertec_am6817/BertecAM6817.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <linux/serial.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <stdexcept>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace bertec
{

namespace
{
constexpr uint8_t kSyncByte = 0x7E;
constexpr uint8_t kEscByte = 0x7D;
constexpr uint8_t kEscXor = 0x20;
constexpr std::size_t kPayloadSize = 17; // 1 byte seq + 8 * uint16 LE
constexpr std::size_t kMaxRxBufBytes = 16384;
constexpr int kStreamPollTimeoutMs = 100;
constexpr int kReconnectIntervalMs = 200;

constexpr uint8_t kCmdStopPing = 0x01;
constexpr uint8_t kCmdGetInfo = 0x02;
constexpr uint8_t kCmdGetCalibration = 0x03;
constexpr uint8_t kCmdStartStream[2] = {0x06, 0x00};
constexpr const char * kBertecByIdPath = "/dev/serial/by-id/usb-Bertec_Corporation_AM6817-E-if00-port0";

uint16_t readU16LE(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

bool unescapeHdlc(const uint8_t * data, std::size_t len, std::vector<uint8_t> & out)
{
  out.clear();
  out.reserve(len);
  for(std::size_t i = 0; i < len; ++i)
  {
    if(data[i] == kEscByte)
    {
      if(i + 1 >= len)
      {
        return false;
      }
      out.push_back(data[i + 1] ^ kEscXor);
      ++i;
    }
    else
    {
      out.push_back(data[i]);
    }
  }
  return true;
}

CalibrationData parseCalibrationText(const std::string & text)
{
  CalibrationData cal;
  std::istringstream iss(text);
  std::string line;
  std::string section;
  std::size_t matrix_row = 0;
  bool first_line = true;

  while(std::getline(iss, line))
  {
    const auto start = line.find_first_not_of(" \t\r\n");
    if(start == std::string::npos)
    {
      continue;
    }
    const auto end = line.find_last_not_of(" \t\r\n");
    line = line.substr(start, end - start + 1);

    if(first_line)
    {
      cal.serial = line;
      first_line = false;
      continue;
    }

    if(line.front() == '[' && line.back() == ']')
    {
      section = line.substr(1, line.size() - 2);
      std::transform(section.begin(), section.end(), section.begin(), ::tolower);
      continue;
    }

    if(section == "calibration matrix")
    {
      if(matrix_row < 6)
      {
        std::istringstream row_ss(line);
        std::string label;
        row_ss >> label;
        for(std::size_t col = 0; col < 8; ++col)
        {
          if(!(row_ss >> cal.matrix[matrix_row][col]))
          {
            throw std::runtime_error("Malformed calibration matrix row: " + line);
          }
        }
        ++matrix_row;
      }
    }
    else if(section == "ranges")
    {
      std::istringstream rng_ss(line);
      double v = 0.0;
      while(rng_ss >> v)
      {
        cal.ranges.push_back(v);
      }
    }
    else if(section == "nameplate")
    {
      if(line.rfind("M ", 0) == 0)
      {
        cal.model = line.substr(2);
      }
      else if(line.rfind("G ", 0) == 0)
      {
        std::istringstream dim_ss(line.substr(2));
        double d = 0.0;
        while(dim_ss >> d)
        {
          cal.dimensions_mm.push_back(d);
        }
      }
    }
  }

  if(matrix_row != 6)
  {
    throw std::runtime_error("Expected 6 calibration matrix rows, got " + std::to_string(matrix_row));
  }
  cal.valid = true;
  return cal;
}

} // namespace

BertecAM6817::BertecAM6817(std::string port, int timeout_ms)
: port_(std::move(port)), active_port_(port_), timeout_ms_(timeout_ms)
{
  rx_buf_.reserve(4096);
}

BertecAM6817::~BertecAM6817()
{
  close();
}

std::string BertecAM6817::resolveDevicePath() const
{
  std::error_code ec;

  // 1. If configured port exists and is explicitly a Bertec AM6817 device, use it
  auto is_bertec_tty = [](const std::string & dev_path) -> bool {
    std::error_code local_ec;
    if(!std::filesystem::exists(dev_path, local_ec))
    {
      return false;
    }
    const auto canon = std::filesystem::weakly_canonical(dev_path, local_ec);
    const auto tty_name = (local_ec ? std::filesystem::path(dev_path) : canon).filename().string();
    std::ifstream prod("/sys/bus/usb-serial/devices/" + tty_name + "/../../product");
    std::string line;
    if(prod && std::getline(prod, line) && line.find("AM6817") != std::string::npos)
    {
      return true;
    }
    return false;
  };

  if(!port_.empty() && port_ != "auto" && is_bertec_tty(port_))
  {
    const auto canon = std::filesystem::weakly_canonical(port_, ec);
    return ec ? port_ : canon.string();
  }

  // 2. Check /dev/serial/by-id symlink for Bertec AM6817-E
  if(std::filesystem::exists(kBertecByIdPath, ec))
  {
    const auto canon = std::filesystem::weakly_canonical(kBertecByIdPath, ec);
    return ec ? std::string(kBertecByIdPath) : canon.string();
  }
  if(std::filesystem::exists("/dev/serial/by-id", ec))
  {
    for(const auto & entry : std::filesystem::directory_iterator("/dev/serial/by-id", ec))
    {
      const auto name = entry.path().filename().string();
      if(name.find("Bertec") != std::string::npos || name.find("AM6817") != std::string::npos)
      {
        const auto canon = std::filesystem::weakly_canonical(entry.path(), ec);
        return ec ? entry.path().string() : canon.string();
      }
    }
  }

  // 3. Scan /sys/bus/usb-serial/devices/ttyUSB* for product "AM6817"
  if(std::filesystem::exists("/sys/bus/usb-serial/devices", ec))
  {
    for(const auto & entry : std::filesystem::directory_iterator("/sys/bus/usb-serial/devices", ec))
    {
      const auto tty_name = entry.path().filename().string();
      const std::string candidate = "/dev/" + tty_name;
      if(is_bertec_tty(candidate))
      {
        return candidate;
      }
    }
  }

  // 4. Fallback to configured port if it exists
  if(!port_.empty() && port_ != "auto" && std::filesystem::exists(port_, ec))
  {
    return port_;
  }
  return port_.empty() ? "/dev/ttyUSB0" : port_;
}

std::string BertecAM6817::activePort() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return active_port_;
}

void BertecAM6817::closeFdOnly() noexcept
{
  if(fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
  rx_buf_.clear();
}

void BertecAM6817::open()
{
  if(fd_ >= 0)
  {
    return;
  }

  const std::string resolved = resolveDevicePath();
  fd_ = ::open(resolved.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd_ < 0)
  {
    throw std::runtime_error("Failed to open " + resolved + ": " + std::strerror(errno));
  }
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    active_port_ = resolved;
  }

  // Attempt to enable FTDI 1 ms low-latency mode before configuring termios baud rate
  if(latencyTimerMs() > 1)
  {
    serial_struct ser{};
    if(::ioctl(fd_, TIOCGSERIAL, &ser) == 0)
    {
      ser.flags |= ASYNC_LOW_LATENCY;
      ::ioctl(fd_, TIOCSSERIAL, &ser);
    }
    if(latencyTimerMs() > 1)
    {
      std::error_code ec;
      const auto tty_name = std::filesystem::weakly_canonical(resolved, ec).filename().string();
      if(!tty_name.empty())
      {
        std::ofstream ofs("/sys/bus/usb-serial/devices/" + tty_name + "/latency_timer");
        if(ofs)
        {
          ofs << "1\n";
        }
      }
    }
  }

  termios attrs{};
  if(::tcgetattr(fd_, &attrs) != 0)
  {
    closeFdOnly();
    throw std::runtime_error("tcgetattr failed on " + resolved + ": " + std::strerror(errno));
  }

  attrs.c_iflag = 0;
  attrs.c_oflag = 0;
  attrs.c_cflag = CS8 | CREAD | CLOCAL;
  attrs.c_lflag = 0;
  attrs.c_cc[VMIN] = 0;
  attrs.c_cc[VTIME] = 0;

  if(::cfsetispeed(&attrs, B1500000) != 0 || ::cfsetospeed(&attrs, B1500000) != 0)
  {
    closeFdOnly();
    throw std::runtime_error("Failed to set 1500000 baud on " + resolved);
  }

  if(::tcsetattr(fd_, TCSANOW, &attrs) != 0)
  {
    closeFdOnly();
    throw std::runtime_error("tcsetattr failed on " + resolved + ": " + std::strerror(errno));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stopStream();
}

int BertecAM6817::latencyTimerMs() const
{
  std::error_code ec;
  const std::string path = (fd_ >= 0) ? activePort() : resolveDevicePath();
  const auto tty_name = std::filesystem::weakly_canonical(path, ec).filename().string();
  if(tty_name.empty())
  {
    return -1;
  }
  std::ifstream ifs("/sys/bus/usb-serial/devices/" + tty_name + "/latency_timer");
  int val = -1;
  if(ifs >> val)
  {
    return val;
  }
  return -1;
}

void BertecAM6817::close()
{
  stopAsync();
  if(fd_ >= 0)
  {
    try
    {
      stopStream();
    }
    catch(...)
    {
    }
    closeFdOnly();
  }
}

std::vector<uint8_t> BertecAM6817::readExact(std::size_t n, int timeout_ms)
{
  if(fd_ < 0)
  {
    throw std::runtime_error("Device is not open");
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  while(rx_buf_.size() < n)
  {
    const auto now = std::chrono::steady_clock::now();
    if(now >= deadline)
    {
      throw std::runtime_error("Timed out waiting for " + std::to_string(n) + " bytes (got " +
                               std::to_string(rx_buf_.size()) + ")");
    }
    const int rem_ms = static_cast<int>(
        std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, std::max(1, rem_ms));
    if(rc < 0)
    {
      if(errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error("poll failed: " + std::string(std::strerror(errno)));
    }
    if(rc == 0)
    {
      continue;
    }
    if(pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
    {
      throw std::runtime_error("Serial device disconnected (poll revents=" + std::to_string(pfd.revents) + ")");
    }

    uint8_t buf[4096];
    const ssize_t got = ::read(fd_, buf, sizeof(buf));
    if(got > 0)
    {
      rx_buf_.insert(rx_buf_.end(), buf, buf + got);
    }
    else if(got < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
    {
      throw std::runtime_error("Serial read failed or device disconnected");
    }
  }

  std::vector<uint8_t> out(rx_buf_.begin(), rx_buf_.begin() + static_cast<std::ptrdiff_t>(n));
  rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + static_cast<std::ptrdiff_t>(n));
  return out;
}

void BertecAM6817::stopStream()
{
  if(fd_ < 0)
  {
    return;
  }
  const ssize_t written = ::write(fd_, &kCmdStopPing, 1);
  (void)written;
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ::tcflush(fd_, TCIFLUSH);
  rx_buf_.clear();
}

DeviceInfo BertecAM6817::getInfo()
{
  stopStream();
  if(::write(fd_, &kCmdGetInfo, 1) != 1)
  {
    throw std::runtime_error("Failed to write getInfo command");
  }
  const auto resp = readExact(24, timeout_ms_);
  if(resp[0] != 0x01)
  {
    throw std::runtime_error("Unexpected ACK for getInfo");
  }

  DeviceInfo info;
  info.firmware_raw = readU16LE(&resp[1]);
  info.eeprom_size = readU16LE(&resp[3]);
  info.sample_rate_hz = readU16LE(&resp[5]);
  info.num_channels = resp[7];

  std::string serial(reinterpret_cast<const char *>(&resp[8]), 16);
  const auto nul = serial.find('\0');
  if(nul != std::string::npos)
  {
    serial.resize(nul);
  }
  info.serial = serial;
  info_ = info;
  return info;
}

CalibrationData BertecAM6817::readCalibration()
{
  stopStream();
  if(::write(fd_, &kCmdGetCalibration, 1) != 1)
  {
    throw std::runtime_error("Failed to write readCalibration command");
  }
  const auto resp = readExact(1025, std::max(timeout_ms_, 1000));
  if(resp[0] != 0x01)
  {
    throw std::runtime_error("Unexpected ACK for readCalibration");
  }

  std::string text(reinterpret_cast<const char *>(&resp[1]), 1024);
  const auto nul = text.find('\0');
  if(nul != std::string::npos)
  {
    text.resize(nul);
  }
  calibration_ = parseCalibrationText(text);
  return calibration_;
}

void BertecAM6817::startStream()
{
  stopStream();
  has_prev_seq_ = false;
  if(::write(fd_, kCmdStartStream, sizeof(kCmdStartStream)) != sizeof(kCmdStartStream))
  {
    throw std::runtime_error("Failed to write startStream command");
  }
  const auto ack = readExact(1, timeout_ms_);
  if(ack[0] != 0x01)
  {
    throw std::runtime_error("Failed to start stream (unexpected ACK)");
  }
}

bool BertecAM6817::tryPopFrame(std::uint8_t & seq_out, std::array<uint16_t, 8> & raw_out)
{
  std::vector<uint8_t> payload;
  while(true)
  {
    auto it_start = std::find(rx_buf_.begin(), rx_buf_.end(), kSyncByte);
    auto it_end = (it_start != rx_buf_.end()) ? std::find(it_start + 1, rx_buf_.end(), kSyncByte) : rx_buf_.end();

    if(it_start == rx_buf_.end() || it_end == rx_buf_.end())
    {
      if(it_start != rx_buf_.begin() && it_start != rx_buf_.end())
      {
        rx_buf_.erase(rx_buf_.begin(), it_start);
      }
      else if(it_start == rx_buf_.end() && rx_buf_.size() > kMaxRxBufBytes)
      {
        rx_buf_.clear();
      }
      return false;
    }

    const std::size_t frame_len = static_cast<std::size_t>(it_end - (it_start + 1));
    const bool ok = unescapeHdlc(&(*(it_start + 1)), frame_len, payload);
    rx_buf_.erase(rx_buf_.begin(), it_end);

    if(!ok || payload.size() != kPayloadSize)
    {
      continue;
    }

    seq_out = payload[0];
    for(std::size_t i = 0; i < 8; ++i)
    {
      raw_out[i] = readU16LE(&payload[1 + 2 * i]);
    }
    return true;
  }
}

void BertecAM6817::drainAvailableNonBlocking()
{
  uint8_t buf[4096];
  while(true)
  {
    const ssize_t got = ::read(fd_, buf, sizeof(buf));
    if(got > 0)
    {
      rx_buf_.insert(rx_buf_.end(), buf, buf + got);
      if(rx_buf_.size() > kMaxRxBufBytes)
      {
        break;
      }
    }
    else if(got == 0 || (got < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)))
    {
      break;
    }
    else
    {
      throw std::runtime_error("Serial read error during stream: " + std::string(std::strerror(errno)));
    }
  }
}

Sample BertecAM6817::readSample()
{
  uint8_t seq = 0;
  std::array<uint16_t, 8> raw{};

  while(!tryPopFrame(seq, raw))
  {
    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeout_ms_);
    if(rc < 0)
    {
      if(errno == EINTR)
      {
        continue;
      }
      throw std::runtime_error("poll failed in readSample: " + std::string(std::strerror(errno)));
    }
    if(rc == 0)
    {
      throw std::runtime_error("Timed out waiting for stream frame");
    }
    if(pfd.revents & (POLLERR | POLLHUP | POLLNVAL))
    {
      throw std::runtime_error("Serial device disconnected (poll revents=" + std::to_string(pfd.revents) + ")");
    }
    drainAvailableNonBlocking();
  }

  Sample s;
  s.seq = seq;
  s.raw = raw;
  s.timestamp = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    s.wrench = convertRawUnlocked(s.raw);
  }
  return s;
}

std::array<double, 6> BertecAM6817::convertRawUnlocked(const std::array<uint16_t, 8> & raw) const
{
  std::array<double, 6> out{};
  if(!calibration_.valid)
  {
    return out;
  }
  std::array<double, 8> delta{};
  for(std::size_t i = 0; i < 8; ++i)
  {
    delta[i] = static_cast<double>(raw[i]) - baseline_[i];
  }
  for(std::size_t r = 0; r < 6; ++r)
  {
    double sum = 0.0;
    for(std::size_t c = 0; c < 8; ++c)
    {
      sum += calibration_.matrix[r][c] * delta[c];
    }
    out[r] = sum;
  }
  return out;
}

bool BertecAM6817::isWrenchWithinSanityLimits(const std::array<double, 6> & w) const noexcept
{
  if(max_range_multiplier_ <= 0.0 || calibration_.ranges.size() < 6)
  {
    return true;
  }
  for(std::size_t i = 0; i < 6; ++i)
  {
    const double limit = max_range_multiplier_ * calibration_.ranges[i];
    if(!std::isfinite(w[i]) || (limit > 0.0 && std::abs(w[i]) > limit))
    {
      return false;
    }
  }
  return true;
}

void BertecAM6817::tareBlocking(std::size_t n_samples)
{
  if(n_samples == 0)
  {
    return;
  }
  startStream();
  std::array<double, 8> accum{};
  try
  {
    for(std::size_t i = 0; i < n_samples; ++i)
    {
      const auto s = readSample();
      for(std::size_t c = 0; c < 8; ++c)
      {
        accum[c] += static_cast<double>(s.raw[c]);
      }
    }
  }
  catch(...)
  {
    stopStream();
    throw;
  }
  stopStream();

  std::lock_guard<std::mutex> lock(data_mutex_);
  for(std::size_t c = 0; c < 8; ++c)
  {
    baseline_[c] = accum[c] / static_cast<double>(n_samples);
  }
  has_baseline_ = true;
}

void BertecAM6817::startAsync(std::size_t initial_tare_samples)
{
  if(running_.load())
  {
    return;
  }
  initial_tare_samples_ = initial_tare_samples;
  if(fd_ >= 0)
  {
    try
    {
      startStream();
      stream_healthy_.store(true);
    }
    catch(...)
    {
      closeFdOnly();
      stream_healthy_.store(false);
    }
  }
  else
  {
    stream_healthy_.store(false);
  }
  running_.store(true);
  reader_thread_ = std::thread(&BertecAM6817::readerLoop, this);
}

void BertecAM6817::stopAsync()
{
  stream_healthy_.store(false);
  if(!running_.exchange(false))
  {
    return;
  }
  if(reader_thread_.joinable())
  {
    reader_thread_.join();
  }
  stopStream();
}

void BertecAM6817::requestTare(std::size_t n_samples)
{
  if(n_samples == 0)
  {
    return;
  }
  std::lock_guard<std::mutex> lock(data_mutex_);
  tare_accum_.fill(0.0);
  tare_total_ = n_samples;
  tare_remaining_ = n_samples;
}

bool BertecAM6817::isTaring() const noexcept
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return tare_remaining_ > 0;
}

Sample BertecAM6817::latestSample() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return latest_;
}

bool BertecAM6817::tryReconnect()
{
  closeFdOnly();
  try
  {
    open();
    if(!calibration_.valid)
    {
      getInfo();
      readCalibration();
    }
    if(!has_baseline_ && initial_tare_samples_ > 0)
    {
      requestTare(initial_tare_samples_);
    }
    startStream();
    return true;
  }
  catch(...)
  {
    closeFdOnly();
    return false;
  }
}

void BertecAM6817::readerLoop()
{
  // Best-effort attempt to elevate reader thread to SCHED_FIFO real-time priority
  sched_param sch{};
  sch.sched_priority = 50;
  ::pthread_setschedparam(::pthread_self(), SCHED_FIFO, &sch);

  uint64_t count = 0;
  uint64_t dropped = 0;
  uint64_t corrupted = 0;
  uint64_t reconnects = 0;

  while(running_.load())
  {
    if(fd_ < 0)
    {
      stream_healthy_.store(false);
      if(tryReconnect())
      {
        ++reconnects;
      }
      else
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(kReconnectIntervalMs));
      }
      continue;
    }

    try
    {
      pollfd pfd{fd_, POLLIN, 0};
      const int rc = ::poll(&pfd, 1, kStreamPollTimeoutMs);
      if(rc < 0)
      {
        if(errno == EINTR)
        {
          continue;
        }
        throw std::runtime_error("poll failed in readerLoop");
      }
      if(rc == 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
      {
        // Close fd immediately so Linux releases the ttyUSB minor node before the cable is plugged back in
        stream_healthy_.store(false);
        closeFdOnly();
        continue;
      }

      // Drain all available bytes from the kernel buffer so we never fall behind
      drainAvailableNonBlocking();
      const auto now = std::chrono::steady_clock::now();

      uint8_t seq = 0;
      std::array<uint16_t, 8> raw{};
      bool got_any = false;
      Sample newest{};

      std::lock_guard<std::mutex> lock(data_mutex_);
      while(tryPopFrame(seq, raw))
      {
        if(has_prev_seq_)
        {
          const uint8_t expected = static_cast<uint8_t>(prev_seq_ + 1);
          if(seq != expected)
          {
            dropped += static_cast<uint8_t>(seq - expected);
          }
        }
        prev_seq_ = seq;
        has_prev_seq_ = true;

        if(tare_remaining_ > 0)
        {
          for(std::size_t c = 0; c < 8; ++c)
          {
            tare_accum_[c] += static_cast<double>(raw[c]);
          }
          --tare_remaining_;
          if(tare_remaining_ == 0 && tare_total_ > 0)
          {
            for(std::size_t c = 0; c < 8; ++c)
            {
              baseline_[c] = tare_accum_[c] / static_cast<double>(tare_total_);
            }
            has_baseline_ = true;
          }
        }

        const auto wrench = (tare_remaining_ > 0) ? std::array<double, 6>{} : convertRawUnlocked(raw);
        if(!isWrenchWithinSanityLimits(wrench))
        {
          ++corrupted;
          latest_.corrupted_frames = corrupted;
          continue;
        }

        ++count;
        newest.seq = seq;
        newest.raw = raw;
        newest.wrench = wrench;
        newest.sample_count = count;
        newest.dropped_frames = dropped;
        newest.corrupted_frames = corrupted;
        newest.reconnect_count = reconnects;
        newest.timestamp = now;
        got_any = true;
      }

      if(got_any)
      {
        latest_ = newest;
        stream_healthy_.store(true);
      }
    }
    catch(...)
    {
      stream_healthy_.store(false);
      closeFdOnly();
      if(!running_.load())
      {
        break;
      }
    }
  }
}

} // namespace bertec
