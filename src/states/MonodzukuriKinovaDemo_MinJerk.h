#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/OrientationTask.h>

struct MonodzukuriKinovaDemo_MinJerk : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:

  void controlModeManager(mc_control::fsm::Controller & ctl);
  bool isTorqueControl_ = false;
  Eigen::Vector3d initPos_;
  bool init_;
  // bool gui_switch_;

  std::shared_ptr<mc_tasks::OrientationTask> oriTask_;
};