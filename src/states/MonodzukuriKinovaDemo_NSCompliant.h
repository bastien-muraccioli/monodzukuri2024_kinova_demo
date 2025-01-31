#pragma once

#include <mc_control/fsm/State.h>
// #include <mc_tasks/AdmittanceTask.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tvm/Robot.h>

struct MonodzukuriKinovaDemo_NSCompliant : mc_control::fsm::State {

  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &ctl) override;

  bool run(mc_control::fsm::Controller &ctl) override;

  void teardown(mc_control::fsm::Controller &ctl) override;

private:
  // std::shared_ptr<mc_tasks::force::AdmittanceTask> admittance_task;

  void controlModeManager(mc_control::fsm::Controller &ctl);
  void dualComplianceControl(mc_control::fsm::Controller &ctl);
  void dualComplianceLoop(mc_control::fsm::Controller &ctl);
  void nullSpaceControl(mc_control::fsm::Controller &ctl);
  void setPositionControl(mc_control::fsm::Controller &ctl);
  bool changeModeRequest_ = false;

  bool dualComplianceFlag_ = false;
  bool nsCompliantFlag_ = true;
  bool eeCompliantFlag_ = false;
  bool dualComplianceLoopFlag_ = false;
  double dualComplianceMaxThreshold_ = 9.0;
  double dualComplianceMinThreshold_ = 5.0;
  double currentForce_ = 0.0;

  bool isPositionControl_ = false;
  bool start_moving_ = false;
  double transitionTime_ = 0.0;
  double transitionDuration_ = 1.0;
  bool transitionStarted_ = false;

  double t;

  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;

  mc_rbdyn::Robot *realRobot;
};
