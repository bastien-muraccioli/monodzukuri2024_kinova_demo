#include "MonodzukuriKinovaDemo_Initial.h"

#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_Initial::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

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
  if(ctl.compPostureTask->eval().norm() < 0.05)
  {
    if(ctl.changeModeAvailable)
    {
      if(ctl.joypadNullSpaceModeFlag)
      {
        ctl.changeModeAvailable = false;
        ctl.changeModeRequest = false;
        output("NS");
        return true;
      }
      else if(ctl.joypadCompliSinusModeFlag)
      {
        ctl.changeModeAvailable = false;
        ctl.changeModeRequest = false;
        output("SIN");
        return true;
      }
      else if(ctl.joypadComplianceModeFlag)
      {
        ctl.changeModeAvailable = false;
        ctl.changeModeRequest = false;
        output("COMPLI");
        return true;
      }
      else if(ctl.joypadMinJerkModeFlag)
      {
        ctl.changeModeAvailable = false;
        ctl.changeModeRequest = false;
        output("MINJERK");
        return true;
      }
    }
  }
  return false;
}

void MonodzukuriKinovaDemo_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_Initial", MonodzukuriKinovaDemo_Initial)
