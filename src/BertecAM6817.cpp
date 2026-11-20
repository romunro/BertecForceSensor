#include <bertec_am6817/BertecAM6817.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sstream>
#include <stdexcept>
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

constexpr uint8_t kCmdStopPing = 0x01;
constexpr uint8_t kCmdGetInfo = 0x02;
constexpr uint8_t kCmdGetCalibration = 0x03;
constexpr uint8_t kCmdStartStream[2] = {0x06, 0x00};

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
    // Trim whitespace
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
: port_(std::move(port)), timeout_ms_(timeout_ms)
{
}

BertecAM6817::~BertecAM6817()
{
  close();
}

void BertecAM6817::open()
{
  if(fd_ >= 0)
  {
    return;
  }

  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if(fd_ < 0)
  {
    throw std::runtime_error("Failed to open " + port_ + ": " + std::strerror(errno));
  }

  termios attrs{};
  if(::tcgetattr(fd_, &attrs) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcgetattr failed on " + port_ + ": " + std::strerror(errno));
  }

  attrs.c_iflag = 0;
  attrs.c_oflag = 0;
  attrs.c_cflag = CS8 | CREAD | CLOCAL;
  attrs.c_lflag = 0;
  attrs.c_cc[VMIN] = 0;
  attrs.c_cc[VTIME] = 0;

  if(::cfsetispeed(&attrs, B1500000) != 0 || ::cfsetospeed(&attrs, B1500000) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("Failed to set 1500000 baud on " + port_);
  }

  if(::tcsetattr(fd_, TCSANOW, &attrs) != 0)
  {
    ::close(fd_);
    fd_ = -1;
    throw std::runtime_error("tcsetattr failed on " + port_ + ": " + std::strerror(errno));
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stopStream();
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
    ::close(fd_);
    fd_ = -1;
  }
  rx_buf_.clear();
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

    uint8_t buf[4096];
    const ssize_t got = ::read(fd_, buf, sizeof(buf));
    if(got > 0)
    {
      rx_buf_.insert(rx_buf_.end(), buf, buf + got);
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

Sample BertecAM6817::readSample()
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
      uint8_t buf[4096];
      const ssize_t got = ::read(fd_, buf, sizeof(buf));
      if(got > 0)
      {
        rx_buf_.insert(rx_buf_.end(), buf, buf + got);
      }
      continue;
    }

    const std::size_t frame_len = static_cast<std::size_t>(it_end - (it_start + 1));
    const bool ok = unescapeHdlc(&(*(it_start + 1)), frame_len, payload);
    rx_buf_.erase(rx_buf_.begin(), it_end);

    if(!ok || payload.size() != kPayloadSize)
    {
      continue;
    }

    Sample s;
    s.seq = payload[0];
    for(std::size_t i = 0; i < 8; ++i)
    {
      s.raw[i] = readU16LE(&payload[1 + 2 * i]);
    }
    s.wrench = convertRaw(s.raw);
    return s;
  }
}

std::array<double, 6> BertecAM6817::convertRaw(const std::array<uint16_t, 8> & raw) const
{
  std::array<double, 6> out{};
  if(!calibration_.valid)
  {
    return out;
  }
  std::array<double, 8> delta{};
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for(std::size_t i = 0; i < 8; ++i)
    {
      delta[i] = static_cast<double>(raw[i]) - baseline_[i];
    }
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
}

void BertecAM6817::startAsync()
{
  if(running_.load())
  {
    return;
  }
  startStream();
  running_.store(true);
  reader_thread_ = std::thread(&BertecAM6817::readerLoop, this);
}

void BertecAM6817::stopAsync()
{
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
  tare_accum_.fill(0.0);
  tare_total_ = n_samples;
  tare_remaining_.store(n_samples);
}

Sample BertecAM6817::latestSample() const
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  return latest_;
}

void BertecAM6817::readerLoop()
{
  uint64_t count = 0;
  uint64_t dropped = 0;
  while(running_.load())
  {
    try
    {
      Sample s = readSample();
      if(has_prev_seq_)
      {
        const uint8_t expected = static_cast<uint8_t>(prev_seq_ + 1);
        if(s.seq != expected)
        {
          dropped += static_cast<uint8_t>(s.seq - expected);
        }
      }
      prev_seq_ = s.seq;
      has_prev_seq_ = true;
      ++count;
      s.sample_count = count;
      s.dropped_frames = dropped;

      const std::size_t rem = tare_remaining_.load();
      if(rem > 0)
      {
        for(std::size_t c = 0; c < 8; ++c)
        {
          tare_accum_[c] += static_cast<double>(s.raw[c]);
        }
        if(rem == 1)
        {
          std::lock_guard<std::mutex> lock(data_mutex_);
          for(std::size_t c = 0; c < 8; ++c)
          {
            baseline_[c] = tare_accum_[c] / static_cast<double>(tare_total_);
          }
          s.wrench = {};
        }
        tare_remaining_.store(rem - 1);
      }

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        latest_ = s;
      }
    }
    catch(...)
    {
      if(!running_.load())
      {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}

} // namespace bertec
