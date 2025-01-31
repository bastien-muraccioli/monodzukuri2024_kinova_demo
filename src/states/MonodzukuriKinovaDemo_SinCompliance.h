#pragma once

#include <mc_control/fsm/State.h>
#include <mc_rtc/gui/Trajectory.h>
#include <mc_tvm/Robot.h>

struct MonodzukuriKinovaDemo_SinCompliance : mc_control::fsm::State {

  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &ctl) override;

  bool run(mc_control::fsm::Controller &ctl) override;

  void teardown(mc_control::fsm::Controller &ctl) override;

private:
  void controlModeManager(mc_control::fsm::Controller &ctl);
  bool isTorqueControl_ = false;
  bool isCompliantControl_ = true;
  bool start_moving_ = false;
  double ctlTime_ = 0;
  double omega_ = 3;
  double R_ = 0.10;
  double minY_ = -0.4;
  double maxY_ = 0.05;
  double yValue_ = 0.0;
  bool yDirection_ = false;
  double transitionTime_ = 0.0;
  double transitionDuration_ = 1.0;
  bool transitionStarted_ = false;
  double posTorqueTransitionTime_ = 0.0;
  double posTorqueTransitionDuration_ = 1.0;
  bool posTorqueTransitionStarted_ = false;
  double init_x = 0.75;
  double init_z = 0.4;

  std::vector<Eigen::Vector3d> visualSinTraj();

  void setPositionControl(mc_control::fsm::Controller &ctl);
  void setTorqueControl(mc_control::fsm::Controller &ctl);
  bool changeModeRequest_ = false;

  mc_rbdyn::Robot *realRobot;
};
