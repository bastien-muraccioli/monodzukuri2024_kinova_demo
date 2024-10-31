#include "MonodzukuriKinovaDemo_MinJerk.h"
#include "../MonodzukuriKinovaDemo.h"

#include <Eigen/src/Geometry/Quaternion.h>

void MonodzukuriKinovaDemo_MinJerk::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_MinJerk::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  auto & robot = ctl.robot();

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
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  initPos_ = robot.bodyPosW("FT_sensor_wrench").translation();

  Eigen::VectorXd W_1(9);
  W_1 << 1e7, 1e7, 1e7, 1e5, 1e5, 1e5, 1e2, 1e2, 1e2;
  // W_1 << 1e2, 1e2, 1e2, 1e3, 1e3, 1e3, 1e1, 1e1, 1e1;
  Eigen::VectorXd W_2(8);
  W_2 << 1e0, 1e0, 1e0, 5 * 1e3, 2 * 1e3, 6 * 1e2, 6 * 1e2, 6 * 1e2;
  // W_2 << 1e0, 1e0, 1e0, 1e2, 1e1, 1e1, 1e1, 1e1;
  ctl.minJerkTask->W_1(W_1);
  ctl.minJerkTask->W_2(W_2);

  ctl.minJerkTask->setTarget(initPos_ + Eigen::Vector3d(0.1, 0.15, 0.0));
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(1.0);
  ctl.compPostureTask->makeCompliant(true);

  ctl.solver().addTask(ctl.minJerkTask);

  oriTask_ = std::make_shared<mc_tasks::OrientationTask>("FT_sensor_wrench", ctl.robots(), ctl.robot().robotIndex(),
                                                         20.0, 10000.0);
  oriTask_->orientation(Eigen::Quaterniond(-0.5, 0.5, 0.5, 0.5).toRotationMatrix());
  ctl.solver().addTask(oriTask_);



  // // Setting gain of posture task for torque control mode
  // ctl.compPostureTask->stiffness(0.0);
  // ctl.compPostureTask->damping(4.0);
  // ctl.compPostureTask->weight(1);

  // ctl.compEETask->reset();
  // ctl.compEETask->positionTask->weight(10000);
  // ctl.compEETask->positionTask->stiffness(10);
  // ctl.compEETask->positionTask->position(ctl.taskPosition_);
  // ctl.compEETask->orientationTask->weight(10000);
  // ctl.compEETask->orientationTask->stiffness(30);
  // ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
  // ctl.solver().addTask(ctl.compEETask);

  // ctl.compPostureTask->makeCompliant(true);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");

  init_ = true;

  // ctl.compPostureTask->reset();
  // ctl.compPostureTask->stiffness(0.5);
  // // ctl.compPostureTask->target(ctl.robot().posture());
  // ctl.compPostureTask->makeCompliant(false);
  // ctl.solver().removeTask(ctl.compEETask);
  // ctl.datastore().assign<std::string>("ControlMode", "Position");

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  mc_rtc::log::success("[MonodzukuriKinovaDemo] Minimum Jerk mode initialized");
}

bool MonodzukuriKinovaDemo_MinJerk::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // mc_rtc::log::info("[Compliance mode] changeModeAvailable: {}, changeModeRequest: {}", ctl.changeModeAvailable, ctl.changeModeRequest);
  if(ctl.changeModeRequest)
  {
    output("OK");
    return true;
  }

   if(ctl.minJerkTask->eval().norm() < 0.03 and ctl.minJerkTask->speed().norm() < 0.001)
  {
    mc_rtc::log::info("Target reached switching target");
    if(init_)
    {
      ctl.minJerkTask->setTarget(initPos_ + Eigen::Vector3d(0.1, -0.15, 0.0));
      init_ = false;
    }
    else
    {
      ctl.minJerkTask->setTarget(initPos_ + Eigen::Vector3d(0.1, 0.15, 0.0));
      init_ = true;
    }
  }

  // controlModeManager(ctl);

  return false;
}

void MonodzukuriKinovaDemo_MinJerk::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.joypadComplianceModeFlag = false;
}

void MonodzukuriKinovaDemo_MinJerk::controlModeManager(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // mc_rtc::log::info("[Compliance mode] joypadTriggerControlFlag: {}, isTorqueControl_: {}", ctl.joypadTriggerControlFlag, isTorqueControl_);
  if(ctl.joypadTriggerControlFlag != isTorqueControl_)
  {
    isTorqueControl_ = !isTorqueControl_;
    if(isTorqueControl_)
    {
      ctl.compPostureTask->stiffness(0.0);
      ctl.compPostureTask->damping(4.0);
      ctl.compPostureTask->weight(1);

      ctl.compEETask->reset();
      ctl.compEETask->positionTask->weight(10000);
      ctl.compEETask->positionTask->stiffness(10);
      ctl.compEETask->positionTask->position(ctl.taskPosition_);
      ctl.compEETask->orientationTask->weight(10000);
      ctl.compEETask->orientationTask->stiffness(30);
      ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
      ctl.solver().addTask(ctl.compEETask);

      ctl.compPostureTask->makeCompliant(true);

      ctl.datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      ctl.compPostureTask->reset();
      ctl.compPostureTask->stiffness(0.5);
      ctl.compPostureTask->makeCompliant(false);
      ctl.solver().removeTask(ctl.compEETask);
      ctl.datastore().assign<std::string>("ControlMode", "Position");
    }
  }
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_MinJerk", MonodzukuriKinovaDemo_MinJerk)