#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tvm/Robot.h>
#include <memory>

struct MonodzukuriKinovaDemo_NSCompliant : mc_control::fsm::State {

  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &ctl) override;

  bool run(mc_control::fsm::Controller &ctl) override;

  void teardown(mc_control::fsm::Controller &ctl) override;

private:

  void addGui(mc_control::fsm::Controller &ctl);
  void addLog(mc_control::fsm::Controller &ctl);

  void controlModeManager(mc_control::fsm::Controller &ctl);
  // void dualComplianceLoop(mc_control::fsm::Controller &ctl);
  void nullSpaceControl(mc_control::fsm::Controller &ctl);
  void setPositionControl(mc_control::fsm::Controller &ctl);
  bool changeToPosCtlRequest_ = false;

  bool dualComplianceFlag_ = false;
  bool nsCompliantFlag_ = true;
  bool eeCompliantFlag_ = false;

  bool isPositionControl_ = false;
  bool start_moving_ = false;
  double transitionTime_ = 0.0;
  double transitionDuration_ = 1.0;
  bool transitionStarted_ = false;

  double t = 0.0;
  std::string tool_frame;

  std::shared_ptr<typename mc_tasks::CompliantEndEffectorTask> compEETask;

  mc_rbdyn::Robot *realRobot;
};
