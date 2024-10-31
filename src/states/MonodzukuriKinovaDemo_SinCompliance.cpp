#include "MonodzukuriKinovaDemo_SinCompliance.h"
#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_SinCompliance::configure(const mc_rtc::Configuration & config) {}

void MonodzukuriKinovaDemo_SinCompliance::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

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

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(4.0);
  ctl.compPostureTask->weight(1);

  ctl.compEETask->reset();
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->positionTask->stiffness(40);
  ctl.compEETask->positionTask->position(ctl.taskPosition_);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(50);
  ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
  ctl.solver().addTask(ctl.compEETask);

  ctl.compPostureTask->makeCompliant(true);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;

  start_moving_ = false;
  ctlTime_=0;
  R_=0.10;
  omega_= 3;
  minY_=-0.4;
  maxY_=0.05;
  yValue_=0.0;
  dt_ = 0.001;
  yDirection_ = true;
  mc_rtc::log::success("[MonodzukuriKinovaDemo] Sinus Compliance mode initialized");
}

bool MonodzukuriKinovaDemo_SinCompliance::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // mc_rtc::log::info("[Compliance mode] changeModeAvailable: {}, changeModeRequest: {}", ctl.changeModeAvailable, ctl.changeModeRequest);
  if(ctl.changeModeRequest)
  {
    output("OK");
    return true;
  }
  ctlTime_ += dt_;

  // Go to the initial position
  if(ctl.compEETask->eval().norm() < 0.20 && !start_moving_)
  {
    mc_rtc::log::info("[Sinus Compliance mode] Start moving");
    start_moving_ = true;
  }

  // Start to do sinus movement
  if (start_moving_ ) 
  { 
    ctl.compEETask->positionTask->position(Eigen::Vector3d(0.68, yValue_, 0.20 + R_*std::sin(omega_*ctlTime_)));
    ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0.5, 0.5));

    yValue_ += (yDirection_ ? 1 : -1) * dt_ / 10;

    if (yValue_ >= maxY_ || yValue_ <= minY_) {
        yDirection_ = !yDirection_;
    }

    // mc_rtc::log::info("[Sinus Compliance mode] yValue: {}", yValue_);
  }
  
  

  // controlModeManager(ctl);

  return false;
}

void MonodzukuriKinovaDemo_SinCompliance::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.joypadComplianceModeFlag = false;
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_SinCompliance", MonodzukuriKinovaDemo_SinCompliance)