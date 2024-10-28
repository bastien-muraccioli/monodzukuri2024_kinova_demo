#include "MonodzukuriKinovaDemo_Initial.h"

#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_Initial::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  // Initialize the velocity damper parameters (closed-loop by default)
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  if (ctl.closeLoopVelocityDamper_)
  {
    mc_rtc::log::info("[RALExpController] Close Loop Velocity damper is enabled");
    ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(ctl.robots(), 0, {0.1, 0.01, 0.5, ctl.m_, ctl.lambda_}, 0.5, true));
  }
  else
  {
    mc_rtc::log::info("[RALExpController] Close Loop Velocity damper is disabled");
    ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(ctl.robots(), 0, ctl.solver().dt(), {0.1, 0.01, 0.5}, 0.5, false, true));
  }
  ctl.solver().addConstraintSet(ctl.dynamicsConstraint);

  // Deactivate feedback from external forces estimator (safer)
  if(ctl.datastore().call<bool>("EF_Estimator::isActive"))
  {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Activate force sensor usage if not used yet
  if(!ctl.datastore().call<bool>("EF_Estimator::useForceSensor"))
  {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain", HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureHome);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  ctl.datastore().assign<std::string>("ControlMode", "Position");
  mc_rtc::log::success("[MonodzukuriKinovaDemo] Switched to Initial state - Position controlled");
}

bool MonodzukuriKinovaDemo_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // if(not ctl.waitingForInput)
  // {
  //   output("OK");
  //   return true;
  // }
  // return false;
  if(ctl.compPostureTask->eval().norm() < 0.05)
  {
    if(ctl.sequenceOutput.compare("FINISHED") == 0)
    {
      ctl.sequenceOutput = "B";
      // ctl.velLimitCounter = 0;
    }

    output(ctl.sequenceOutput);
    return true;
  }
  return false;
}

void MonodzukuriKinovaDemo_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_Initial", MonodzukuriKinovaDemo_Initial)
