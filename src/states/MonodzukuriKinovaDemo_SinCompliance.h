#pragma once

#include <mc_control/fsm/State.h>

struct MonodzukuriKinovaDemo_SinCompliance : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:

  // void controlModeManager(mc_control::fsm::Controller & ctl);
  // bool isTorqueControl_ = false;
  bool start_moving_;
  bool init_;
  double ctlTime_;
  double omega_;
  double R_;
  double minY_;
  double maxY_;
  double yValue_;
  bool yDirection_;
  double dt_ = 0.001;
};