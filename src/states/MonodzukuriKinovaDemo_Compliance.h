#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tvm/Robot.h>

struct MonodzukuriKinovaDemo_Compliance : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:

  void controlModeManager(mc_control::fsm::Controller & ctl);
  void setTorqueControl(mc_control::fsm::Controller & ctl);
  void setPositionControl(mc_control::fsm::Controller & ctl);
  bool isTorqueControl_ = false;
  double transitionTime_= 0.0;
  double transitionDuration_= 1.0;
  bool transitionStarted_= false;
  bool changeModeRequest_ = false;
};