#include "MonodzukuriKinovaDemo_SinCompliance.h"
#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_SinCompliance::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_SinCompliance::start(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Enable feedback from external forces estimator (safer)
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Enable force sensor usage if not active
  if (!ctl.datastore().call<bool>("EF_Estimator::useForceSensor")) {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain",
                                     HIGH_RESIDUAL_GAIN);

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureTarget);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.compliantFlag = true;

  ctl.game.setControlMode(6);

  mc_rtc::log::success(
      "[MonodzukuriKinovaDemo] Sinus Compliance mode initialized");
}

bool MonodzukuriKinovaDemo_SinCompliance::run(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  if (ctl.changeModeRequest) {
    transitionTime_ += ctl.dt_ctrl;
    if (!transitionStarted_) {
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      transitionStarted_ = true;
    }
    if (transitionTime_ > transitionDuration_) {
      output("OK");
      return true;
    }
  }

  // Go to the initial position
  if (ctl.compPostureTask->eval().norm() < 0.05 && !start_moving_ &&
      !transitionStarted_) {
    mc_rtc::log::info("[Sinus Compliance mode] Start moving");
    start_moving_ = true;
    ctl.compEETask->reset();
    ctl.compEETask->positionTask->weight(10000);
    ctl.compEETask->positionTask->stiffness(10);
    ctl.compEETask->positionTask->position(ctl.taskPosition_);
    ctl.compEETask->orientationTask->weight(10000);
    ctl.compEETask->orientationTask->stiffness(10);
    ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
    ctl.compEETask->makeCompliant(false);
    ctl.solver().addTask(ctl.compEETask);
    ctl.compPostureTask->makeCompliant(false);
    ctl.compPostureTask->stiffness(0.5);
    ctl.datastore().assign<std::string>("ControlMode", "Position");
  }

  if(start_moving_ && changeModeRequest_) {
    if(ctl.robot().tvmRobot().alpha()->value().norm() < 0.01)
    {
      changeModeRequest_ = false;
      setPositionControl(ctl);
    }
  }

  // Start to do sinus movement
  if (start_moving_ && !transitionStarted_ && !changeModeRequest_) {
    controlModeManager(ctl);
    ctlTime_ += ctl.dt_ctrl;
    ctl.compEETask->positionTask->position(
        Eigen::Vector3d(0.68, yValue_, 0.3 + R_ * std::sin(omega_ * ctlTime_)));
    ctl.compEETask->positionTask->refVel(
        Eigen::Vector3d(0, 0.1 * (yDirection_ ? 1 : -1),
                        omega_ * R_ * std::cos(omega_ * ctlTime_)));
    ctl.compEETask->positionTask->refAccel(Eigen::Vector3d(
        0, 0, -omega_ * omega_ * R_ * std::sin(omega_ * ctlTime_)));

    yValue_ += (yDirection_ ? 1 : -1) * ctl.dt_ctrl / 10;

    if (yValue_ >= maxY_ || yValue_ <= minY_) {
      yDirection_ = !yDirection_;
    }
  }

  return false;
}

void MonodzukuriKinovaDemo_SinCompliance::teardown(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

void MonodzukuriKinovaDemo_SinCompliance::controlModeManager(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // If press the Circle button, activate/deactivate the dual compliance
  if (ctl.posTorqueFlag && !isTorqueControl_) {
    mc_rtc::log::info("[Sinus Compliance mode] Torque controlled");
    isTorqueControl_ = true;
    ctl.compEETask->positionTask->stiffness(100);
    ctl.compEETask->orientationTask->stiffness(100);
    ctl.compEETask->makeCompliant(isCompliantControl_);
    ctl.compPostureTask->stiffness(0.0);
    ctl.compPostureTask->damping(5.0);
    ctl.compPostureTask->makeCompliant(true);
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
  } else if (isTorqueControl_ && !ctl.posTorqueFlag) {
    mc_rtc::log::info("[Sinus Compliance mode] Position controlled");
    isTorqueControl_ = false;
    changeModeRequest_ = true;
    ctl.compEETask.reset();
    ctl.compPostureTask->setGains(10.0, 20.0);
  }

  if (ctl.compliantFlag && !isCompliantControl_ && isTorqueControl_) {
    mc_rtc::log::info("[Sinus Compliance mode] EEF Compliant");
    isCompliantControl_ = true;
    ctl.compEETask->makeCompliant(true);
    ctl.game.setControlMode(6);
    
  } else if (!ctl.compliantFlag && isCompliantControl_ && isTorqueControl_) {
    mc_rtc::log::info("[Sinus Compliance mode] EEF no Compliant");
    isCompliantControl_ = false;
    ctl.compEETask->makeCompliant(false);
    ctl.game.setControlMode(0);
  }
}

void MonodzukuriKinovaDemo_SinCompliance::setPositionControl(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.compEETask->positionTask->stiffness(10);
  ctl.compEETask->orientationTask->stiffness(10);
  ctl.compEETask->makeCompliant(false);
  ctl.compPostureTask->makeCompliant(false);
  ctl.compPostureTask->stiffness(0.5);
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_SinCompliance",
                    MonodzukuriKinovaDemo_SinCompliance)
