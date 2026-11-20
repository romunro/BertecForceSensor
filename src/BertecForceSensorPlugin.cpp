#include "BertecForceSensorPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/gui/ArrayLabel.h>
#include <mc_rtc/gui/Button.h>
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/logging.h>

#include <chrono>
#include <functional>

namespace mc_plugin
{

BertecForceSensor::~BertecForceSensor()
{
  if(device_)
  {
    device_->close();
  }
}

mc_control::GlobalPlugin::GlobalPluginConfiguration BertecForceSensor::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

void BertecForceSensor::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  port_ = config("port", std::string("/dev/ttyUSB0"));
  sensor_name_ = config("sensor_name", std::string("BertecForcePlate"));
  robot_name_ = config("robot_name", std::string(""));
  datastore_prefix_ = config("datastore_prefix", std::string("BertecForceSensor"));
  tare_samples_ = config("tare_samples", 200);
  watchdog_timeout_ms_ = config("watchdog_timeout_ms", 20.0);
  max_range_multiplier_ = config("max_range_multiplier", 3.0);
  zero_on_stale_ = config("zero_on_stale", true);
  inject_to_robot_sensor_ = config("inject_to_robot_sensor", true);
  log_raw_ = config("log_raw", false);
  enable_gui_ = config("gui", true);

  device_ = std::make_unique<bertec::BertecAM6817>(port_);
  device_->setMaxRangeMultiplier(max_range_multiplier_);
  try
  {
    device_->open();
    const int latency_ms = device_->latencyTimerMs();
    if(latency_ms > 1)
    {
      mc_rtc::log::warning(
          "[BertecForceSensor] FTDI latency_timer on {} is {} ms (expected 1 ms). "
          "Frames will still stream at 1000 Hz avg, but arrive in ~3-4 ms USB micro-batches. "
          "Run `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` or add a udev rule.",
          port_, latency_ms);
    }
    const auto info = device_->getInfo();
    const auto cal = device_->readCalibration();
    mc_rtc::log::success("[BertecForceSensor] Connected to {} (model: {}, serial: {}, rate: {} Hz)",
                         port_, cal.model, info.serial, info.sample_rate_hz);

    if(tare_samples_ > 0)
    {
      mc_rtc::log::info("[BertecForceSensor] Taring over {} samples...", tare_samples_);
      device_->tareBlocking(static_cast<std::size_t>(tare_samples_));
    }

    device_->startAsync();
    connected_ = true;
    stale_ = false;
  }
  catch(const std::exception & e)
  {
    connected_ = false;
    stale_ = true;
    mc_rtc::log::error("[BertecForceSensor] Failed to initialize on {}: {}", port_, e.what());
  }

  registerDatastore(controller);
  addLog(controller);
  if(enable_gui_)
  {
    addGui(controller);
  }
}

void BertecForceSensor::reset(mc_control::MCGlobalController & controller)
{
  registerDatastore(controller);
  addLog(controller);
  if(enable_gui_)
  {
    addGui(controller);
  }
}

void BertecForceSensor::before(mc_control::MCGlobalController & controller)
{
  updateWrench();

  auto & ctl = controller.controller();
  ctl.datastore().assign<sva::ForceVecd>(datastore_prefix_ + "::wrench", wrench_);
  ctl.datastore().assign<bool>(datastore_prefix_ + "::connected", connected_ && !stale_);
  ctl.datastore().assign<bool>(datastore_prefix_ + "::stale", stale_);

  if(inject_to_robot_sensor_)
  {
    const std::string & rname = robot_name_.empty() ? ctl.robot().name() : robot_name_;
    if(ctl.robots().hasRobot(rname))
    {
      auto & robot = ctl.robot(rname);
      if(robot.hasForceSensor(sensor_name_))
      {
        auto & data = *robot.data();
        data.forceSensors.at(data.forceSensorsIndex.at(sensor_name_)).wrench(wrench_);
      }
    }
    if(ctl.realRobots().hasRobot(rname))
    {
      auto & real_robot = ctl.realRobot(rname);
      if(real_robot.hasForceSensor(sensor_name_))
      {
        auto & data = *real_robot.data();
        data.forceSensors.at(data.forceSensorsIndex.at(sensor_name_)).wrench(wrench_);
      }
    }
  }
}

void BertecForceSensor::after(mc_control::MCGlobalController & /*controller*/) {}

void BertecForceSensor::updateWrench()
{
  if(!connected_ || !device_)
  {
    stale_ = true;
    if(zero_on_stale_)
    {
      wrench_ = sva::ForceVecd::Zero();
    }
    return;
  }

  const auto s = device_->latestSample();
  sample_count_ = s.sample_count;
  dropped_frames_ = s.dropped_frames;
  corrupted_frames_ = s.corrupted_frames;

  if(s.sample_count == 0)
  {
    stale_ = true;
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  sample_age_ms_ = std::chrono::duration<double, std::milli>(now - s.timestamp).count();

  const bool healthy = device_->isStreamHealthy() &&
                       (watchdog_timeout_ms_ <= 0.0 || sample_age_ms_ <= watchdog_timeout_ms_);

  if(!healthy)
  {
    stale_ = true;
    if(!stale_warned_)
    {
      mc_rtc::log::error("[BertecForceSensor] Sensor stream stale or disconnected (age: {:.1f} ms > {:.1f} ms)",
                         sample_age_ms_, watchdog_timeout_ms_);
      stale_warned_ = true;
    }
    if(zero_on_stale_)
    {
      wrench_ = sva::ForceVecd::Zero();
    }
    return;
  }

  if(stale_warned_)
  {
    mc_rtc::log::success("[BertecForceSensor] Sensor stream recovered (age: {:.2f} ms)", sample_age_ms_);
    stale_warned_ = false;
  }
  stale_ = false;

  wrench_ = sva::ForceVecd(Eigen::Vector3d(s.wrench[3], s.wrench[4], s.wrench[5]),
                           Eigen::Vector3d(s.wrench[0], s.wrench[1], s.wrench[2]));
  if(log_raw_)
  {
    for(Eigen::Index i = 0; i < 8; ++i)
    {
      raw_adc_(i) = static_cast<double>(s.raw[static_cast<std::size_t>(i)]);
    }
  }
}

void BertecForceSensor::registerDatastore(mc_control::MCGlobalController & controller)
{
  auto & ds = controller.controller().datastore();
  if(!ds.has(datastore_prefix_ + "::wrench"))
  {
    ds.make<sva::ForceVecd>(datastore_prefix_ + "::wrench", wrench_);
  }
  if(!ds.has(datastore_prefix_ + "::connected"))
  {
    ds.make<bool>(datastore_prefix_ + "::connected", connected_ && !stale_);
  }
  if(!ds.has(datastore_prefix_ + "::stale"))
  {
    ds.make<bool>(datastore_prefix_ + "::stale", stale_);
  }
  if(!ds.has(datastore_prefix_ + "::tare"))
  {
    ds.make<std::function<void()>>(datastore_prefix_ + "::tare", [this]() {
      if(device_ && connected_)
      {
        device_->requestTare(static_cast<std::size_t>(std::max(1, tare_samples_)));
      }
    });
  }
}

void BertecForceSensor::addLog(mc_control::MCGlobalController & controller)
{
  auto & logger = controller.controller().logger();
  logger.addLogEntry(sensor_name_ + "_wrench", [this]() -> const sva::ForceVecd & { return wrench_; });
  logger.addLogEntry(sensor_name_ + "_sample_count", [this]() { return sample_count_; });
  logger.addLogEntry(sensor_name_ + "_dropped_frames", [this]() { return dropped_frames_; });
  logger.addLogEntry(sensor_name_ + "_corrupted_frames", [this]() { return corrupted_frames_; });
  logger.addLogEntry(sensor_name_ + "_age_ms", [this]() { return sample_age_ms_; });
  logger.addLogEntry(sensor_name_ + "_stale", [this]() { return stale_; });
  if(log_raw_)
  {
    logger.addLogEntry(sensor_name_ + "_raw", [this]() -> const Eigen::VectorXd & { return raw_adc_; });
  }
}

void BertecForceSensor::addGui(mc_control::MCGlobalController & controller)
{
  auto gui = controller.controller().gui();
  if(!gui)
  {
    return;
  }
  gui->addElement(
      {"Plugins", "BertecForceSensor"},
      mc_rtc::gui::Label("Port", [this]() { return port_; }),
      mc_rtc::gui::Label("Status",
                         [this]() {
                           if(!connected_)
                           {
                             return "Disconnected";
                           }
                           return stale_ ? "STALE / TIMEOUT" : "Streaming (OK)";
                         }),
      mc_rtc::gui::ArrayLabel("Force [N]", {"Fx", "Fy", "Fz"},
                              [this]() -> const Eigen::Vector3d & { return wrench_.force(); }),
      mc_rtc::gui::ArrayLabel("Moment [Nm]", {"Mx", "My", "Mz"},
                              [this]() -> const Eigen::Vector3d & { return wrench_.couple(); }),
      mc_rtc::gui::Label("Sample Age [ms]", [this]() { return sample_age_ms_; }),
      mc_rtc::gui::Label("Samples", [this]() { return std::to_string(sample_count_); }),
      mc_rtc::gui::Label("Dropped", [this]() { return std::to_string(dropped_frames_); }),
      mc_rtc::gui::Label("Corrupted", [this]() { return std::to_string(corrupted_frames_); }),
      mc_rtc::gui::Button("Tare", [this]() {
        if(device_ && connected_)
        {
          device_->requestTare(static_cast<std::size_t>(std::max(1, tare_samples_)));
        }
      }));
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("BertecForceSensor", mc_plugin::BertecForceSensor)
