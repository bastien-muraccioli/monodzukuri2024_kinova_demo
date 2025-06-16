#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tvm/Robot.h>

struct MonodzukuriKinovaDemo_wayPointDCompliant : mc_control::fsm::State {

  void configure(const mc_rtc::Configuration &config) override;

  void start(mc_control::fsm::Controller &ctl) override;

  bool run(mc_control::fsm::Controller &ctl) override;

  void teardown(mc_control::fsm::Controller &ctl) override;

  void addGui(mc_control::fsm::Controller &ctl);

private:
  int wayPointIndex_ = 0;
  double stiffness_posture_;
  double damping_posture_;
  double stiffness_task_;
  double damping_task_;
  double stiffnessMin_ = 10.0;
  double stiffnessMax_ = 150.0;
  // double stiffnessMin_ = 5.0;
  // double stiffnessMax_ = 75.0;
  // Parameters for stiffness adjustment
  // stiffness = A* exp(k_slope_ * distance) + C
  double k_slope_ = -2; // Slope for stiffness adjustment (Strictly negative for decreasing stiffness)
  double A_;
  double C_;

  // bool eeTaskHasReachedTarget_ = false;
};
