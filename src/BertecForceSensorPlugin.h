#pragma once

#include <bertec_am6817/BertecAM6817.h>
#include <mc_control/GlobalPlugin.h>

#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/Core>

#include <memory>
#include <string>

namespace mc_plugin
{

struct BertecForceSensor : public mc_control::GlobalPlugin
{
  ~BertecForceSensor() override;

  GlobalPluginConfiguration configuration() override;

  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;
  void reset(mc_control::MCGlobalController & controller) override;
  void before(mc_control::MCGlobalController & controller) override;
  void after(mc_control::MCGlobalController & controller) override;

private:
  void registerDatastore(mc_control::MCGlobalController & controller);
  void addLog(mc_control::MCGlobalController & controller);
  void addGui(mc_control::MCGlobalController & controller);
  void updateWrench();

  std::unique_ptr<bertec::BertecAM6817> device_;

  std::string port_ = "/dev/ttyUSB0";
  std::string sensor_name_ = "BertecForcePlate";
  std::string robot_name_;
  std::string datastore_prefix_ = "BertecForceSensor";
  int tare_samples_ = 200;
  bool inject_to_robot_sensor_ = true;
  bool log_raw_ = false;
  bool enable_gui_ = true;

  bool connected_ = false;
  sva::ForceVecd wrench_ = sva::ForceVecd::Zero();
  uint64_t sample_count_ = 0;
  uint64_t dropped_frames_ = 0;
  Eigen::VectorXd raw_adc_ = Eigen::VectorXd::Zero(8);
};

} // namespace mc_plugin
