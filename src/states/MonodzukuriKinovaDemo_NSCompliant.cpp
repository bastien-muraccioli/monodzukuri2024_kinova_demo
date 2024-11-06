#include "MonodzukuriKinovaDemo_NSCompliant.h"
#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_NSCompliant::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_NSCompliant::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Disable feedback from external forces estimator (safer)
  if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Enable force sensor usage if not active
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  admittance_task = std::make_shared<mc_tasks::force::AdmittanceTask>("FT_sensor_wrench", ctl.robots(), ctl.robot().robotIndex(), 5.0, 10000.0);

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureTarget);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  
  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.activateFlag = false;

  mc_rtc::log::success("[MonodzukuriKinovaDemo] Switched to Sensor Testing state - Position controlled");
}

bool MonodzukuriKinovaDemo_NSCompliant::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Exit State
  if(ctl.changeModeRequest)
  {
    transitionTime_ += ctl.dt_ctrl;
    if(!transitionStarted_)
    {
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      transitionStarted_ = true;
    }
    if(transitionTime_ > transitionDuration_)
    {
      output("OK");
      return true;
    }
  }

  // Initial state
  if(ctl.compPostureTask->eval().norm() < 0.05 && !start_moving_ && !transitionStarted_)
  {
    mc_rtc::log::info("[Null Space mode] Start moving");
    start_moving_ = true;
  }

  // While the state is running
  if(start_moving_ && !transitionStarted_)
  {
    controlModeManager(ctl);
    if(dualComplianceFlag_ && isTorqueControl_)
    {
      dualComplianceLoop(ctl);
    }
  }
  

  return false;
}

void MonodzukuriKinovaDemo_NSCompliant::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.solver().removeTask(admittance_task);
}

void MonodzukuriKinovaDemo_NSCompliant::controlModeManager(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // If press the X button, activate/deactivate the dual compliance
  if(ctl.activateFlag && !dualComplianceFlag_)
  {
    mc_rtc::log::info("[Null Space mode] Dual Compliance activated");
    dualComplianceFlag_ = true;
  }
  else if(dualComplianceFlag_ && !ctl.activateFlag)
  {
    mc_rtc::log::info("[Null Space mode] Dual Compliance deactivated");
    dualComplianceFlag_ = false;
  }

  // If press the R2 trigger, activate torque control, if not, activate position control
  if(ctl.joypadTriggerControlFlag != isTorqueControl_)
  {
    isTorqueControl_ = !isTorqueControl_;
    if(isTorqueControl_)
    {
      if(!dualComplianceFlag_)
      {
        if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
        {
          ctl.datastore().call("EF_Estimator::toggleActive");
        }
        nullSpaceControl(ctl);
      }
      ctl.datastore().assign<std::string>("ControlMode", "Torque");
    }
    else
    {
      // Position control
      ctl.solver().removeTask(admittance_task);
      ctl.solver().removeTask(ctl.compEETask);
      ctl.compPostureTask->reset();
      ctl.compPostureTask->stiffness(0.5);
      ctl.compPostureTask->makeCompliant(false);

      ctl.datastore().assign<std::string>("ControlMode", "Position");
    }
  }
}

void MonodzukuriKinovaDemo_NSCompliant::dualComplianceLoop(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Activate the admittance control if the force is below the threshold
  currentForce_ = ctl.robot().forceSensor("EEForceSensor").force().norm();
  // mc_rtc::log::info("[Null Space mode] Current force: {}", currentForce_);

  if(currentForce_ > dualComplianceThreshold_)
  {
    if(!admittanceFlag_)
    {
      mc_rtc::log::info("Above threshold, admittance control activated");
      if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
      {
        ctl.datastore().call("EF_Estimator::toggleActive");
      }
      admittanceControl(ctl);
      admittanceFlag_ = true;
    }
  }
  else
  {
    if(admittanceFlag_)
    {
      mc_rtc::log::info("Below threshold, null space control activated");
      if(!ctl.datastore().call<bool>("EF_Estimator::isActive"))
      {
        ctl.datastore().call("EF_Estimator::toggleActive");
      }
      nullSpaceControl(ctl);
      admittanceFlag_ = false;
    }
  }
}

void MonodzukuriKinovaDemo_NSCompliant::admittanceControl(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("[Null Space mode] Admittance control");
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  
  ctl.solver().removeTask(ctl.compEETask);
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(5.0);
  ctl.compPostureTask->weight(1);
  ctl.compPostureTask->makeCompliant(true);

  admittance_task->reset();
  admittance_task->admittance(sva::ForceVecd({0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  admittance_task->weight(10000);
  ctl.solver().addTask(admittance_task);
}

void MonodzukuriKinovaDemo_NSCompliant::nullSpaceControl(mc_control::fsm::Controller & ctl_)
{
  mc_rtc::log::info("[Null Space mode] Null Space control");
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  
  ctl.solver().removeTask(admittance_task);

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(5.0);
  ctl.compPostureTask->weight(1);
  ctl.compPostureTask->makeCompliant(true);

  ctl.compEETask->reset();
  ctl.compEETask->positionTask->reset();
  ctl.compEETask->positionTask->stiffness(200);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->reset();
  ctl.compEETask->orientationTask->stiffness(200);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->makeCompliant(false);
  ctl.solver().addTask(ctl.compEETask);
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_NSCompliant", MonodzukuriKinovaDemo_NSCompliant)