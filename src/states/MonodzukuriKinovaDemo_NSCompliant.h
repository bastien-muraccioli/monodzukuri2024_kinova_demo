#pragma once

#include <mc_control/fsm/State.h>
// #include <mc_tasks/PositionTask.h>
// #include <memory>

struct MonodzukuriKinovaDemo_NSCompliant : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  void controlModeManager(mc_control::fsm::Controller & ctl);
  bool isTorqueControl_ = false;
};