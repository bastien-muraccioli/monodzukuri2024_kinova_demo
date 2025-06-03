#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tvm/Robot.h>
#include <vector>
#include <utility>
#include <Eigen/Core>

struct MonodzukuriKinovaDemo_DCompliant : mc_control::fsm::State {

  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &ctl) override;

  bool run(mc_control::fsm::Controller &ctl) override;

  void teardown(mc_control::fsm::Controller &ctl) override;

private:
  void controlModeManager(mc_control::fsm::Controller &ctl);
  void dualComplianceControl(mc_control::fsm::Controller &ctl);
  void addWayPoint(mc_control::fsm::Controller &ctl);
  void removeWayPoint(mc_control::fsm::Controller &ctl);

  bool changeModeRequest_ = false;

  bool nsComplianceStateFlag_ = false;
  bool addWayPointFlag_ = false;
  bool removeWayPointFlag_ = false;
  bool runWayPointFlag_ = false;

  bool dualComplianceLoopFlag_ = false;
  double dualComplianceMaxThreshold_ = 9.0;
  double dualComplianceMinThreshold_ = 5.0;
  double currentForce_ = 0.0;

  bool isPositionControl_ = false;
  bool start_moving_ = false;
  double transitionTime_ = 0.0;
  double transitionDuration_ = 1.0;
  bool transitionStarted_ = false;
};
