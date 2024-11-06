#include "MonodzukuriKinovaDemo_MinJerk.h"
#include "../MonodzukuriKinovaDemo.h"

#include <Eigen/src/Geometry/Quaternion.h>

void MonodzukuriKinovaDemo_MinJerk::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_MinJerk::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // auto & robot = ctl.robot();

  // Disable feedback from external forces estimator (safer)
  if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Enable force sensor usage if not active
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", FITTS_RESIDUAL_GAIN);

  mj_task = std::make_shared<mc_tasks::MinimumJerkTask>("FT_sensor_mounting", ctl.robots(), ctl.robot().robotIndex(),
                                                        10000.0);

  Eigen::Vector3d LQR_Q;
  LQR_Q << 1e8, 1e6, 1e3;
  mj_task->LQR_Q(LQR_Q);
  mj_task->LQR_R(10);
  mj_task->W_e(Eigen::Vector3d({1, 1, 1}));
  mj_task->W_u(Eigen::Vector4d(1, 1000, 1000, 1000));
  mj_task->fitts_b(0.31);
  mj_task->fitts_a(0.19);
  mj_task->react_time(0.25);

  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(1.0);
  ctl.compPostureTask->makeCompliant(true);

  oriTask_ = std::make_shared<mc_tasks::OrientationTask>("FT_sensor_wrench", ctl.robots(), ctl.robot().robotIndex(),
                                                         20.0, 10000.0);
  oriTask_->orientation(Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).toRotationMatrix());
  ctl.solver().addTask(oriTask_);

  init_pose = ctl.robot().bodyPosW("FT_sensor_wrench").translation() + Eigen::Vector3d(0.1, 0.0, 0.0);
  ctl.target_pose.first = init_pose(1);
  ctl.target_pose.second = init_pose(2);

  ctl.compPostureTask->stiffness(100);
  ctl.compPostureTask->makeCompliant(false);

  controlled_frame = &ctl.robot().frame("FT_sensor_mounting");

  ctl.datastore().assign<std::string>("ControlMode", "Torque");

  auto new_target_pos = ctl.game.getTargetPos();
  float new_target_radius = ctl.game.getTargetRadius();
  auto win_size = ctl.game.getWinSize();
  auto robot_radius = ctl.game.getRobotRadius();
  target_circle(0) = ctl.target_pose.first - robot_radius + new_target_pos.first * (2.0 * robot_radius) / win_size.first;
  target_circle(1) = ctl.target_pose.second - robot_radius + new_target_pos.second * (2.0 * robot_radius) / win_size.second;
  double W = 2.0 * new_target_radius * (2.0 * robot_radius) / win_size.first;
  mc_rtc::log::info("New W = {}, radius = {}", W, new_target_radius);
  mj_task->setTarget(Eigen::Vector3d(init_pose(0), target_circle(0), target_circle(1)));
  mj_task->W(W);
  mc_rtc::log::info("Target: {}", Eigen::Vector3d(init_pose(0), target_circle(0), target_circle(1)));

  ctl.activateFlag = false;
  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  mc_rtc::log::success("[MonodzukuriKinovaDemo] Minimum Jerk mode initialized");
}

bool MonodzukuriKinovaDemo_MinJerk::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Exit State
  if(ctl.changeModeRequest)
  {
    transitionTime_ += ctl.dt_ctrl;
    if(!transitionStarted_)
    {
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->weight(10000);
      ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      ctl.solver().addTask(ctl.compEETask);
      transitionStarted_ = true;
    }
    if(transitionTime_ > transitionDuration_)
    {
      output("OK");
      return true;
    }
  }

  // While the state is running
  if(!transitionStarted_)
  {
    if(ctl.activateFlag && !start_moving_)
    {
      ctl.solver().addTask(mj_task);
      start_moving_  = true;
    }

    std::string bodyName = controlled_frame->body();
    sva::PTransformd transform(ctl.robot().bodyPosW(bodyName));
    Eigen::Vector3d pose = ctl.robot().frame("FT_sensor_mounting").position().translation();
    Eigen::Vector3d vel = ctl.robot().frame("FT_sensor_mounting").velocity().linear();
    Eigen::Vector3d acc = transform.rotation().transpose() * ctl.robot().bodyAccB(bodyName).linear()
                          + ctl.robot().bodyVelW(bodyName).angular().cross(vel);
    ctl.game.setRobotPosition(pose(1), pose(2), ctl.target_pose.first, ctl.target_pose.second);
    if(ctl.game.getNewTargetBool())
    {
      save_last_target(ctl);
    }
  }
  return false;
}

void MonodzukuriKinovaDemo_MinJerk::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.game.setRobotRadius(ctl.robot_radius);
  ctl.solver().removeTask(mj_task);
  ctl.solver().removeTask(oriTask_);
  ctl.solver().removeTask(ctl.compEETask);
}

void MonodzukuriKinovaDemo_MinJerk::save_last_target(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  auto robot_radius = ctl.game.getRobotRadius();
  auto win_size = ctl.game.getWinSize();
  auto new_target_pos = ctl.game.getTargetPos();
  float new_target_radius = ctl.game.getTargetRadius();
  target_circle(0) = ctl.target_pose.first - robot_radius + new_target_pos.first * (2.0 * robot_radius) / win_size.first;
  target_circle(1) = ctl.target_pose.second - robot_radius + new_target_pos.second * (2.0 * robot_radius) / win_size.second;
  double W = 2.0 * new_target_radius * (2.0 * robot_radius) / win_size.first;
  mc_rtc::log::info("New W = {}, radius = {}", W, new_target_radius);
  mj_task->setTarget(Eigen::Vector3d(init_pose(0), target_circle(0), target_circle(1)));
  mj_task->W(W);
  projection_vector = target_circle - controlled_frame->position().translation().tail<2>();

  FittsData data = ctl.game.getLastData();
  ctl.game.clearNewTargetBool();
}


EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_MinJerk", MonodzukuriKinovaDemo_MinJerk)