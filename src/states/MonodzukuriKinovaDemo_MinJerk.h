#pragma once

#include <mc_control/fsm/State.h>

#include <mc_tasks/OrientationTask.h>

#include <mc_tasks/MinimumJerkTask.h>

struct MonodzukuriKinovaDemo_MinJerk : mc_control::fsm::State
{

  void configure(const mc_rtc::Configuration & config) override;

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

  void save_last_target(mc_control::fsm::Controller & ctl);

private:
  std::shared_ptr<mc_tasks::MinimumJerkTask> mj_task;
  std::shared_ptr<mc_tasks::OrientationTask> oriTask_;

  Eigen::Vector3d init_pose;
  Eigen::Vector2d target_circle;
  Eigen::Vector2d projection_vector;
  mc_rbdyn::RobotFrame * controlled_frame;

  bool start_moving_=false;

  double transitionTime_= 0.0;
  double transitionDuration_= 1.0;
  bool transitionStarted_= false;
};