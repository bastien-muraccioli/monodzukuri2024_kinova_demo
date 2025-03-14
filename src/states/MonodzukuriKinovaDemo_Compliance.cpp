#include "MonodzukuriKinovaDemo_Compliance.h"
#include "../MonodzukuriKinovaDemo.h"

void MonodzukuriKinovaDemo_Compliance::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_Compliance::start(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Disable feedback from external forces estimator (safer)
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  // Enable force sensor usage if not active
  if (!ctl.datastore().call<bool>("EF_Estimator::useForceSensor")) {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }
  ctl.datastore().call<void, double>("EF_Estimator::setGain",
                                     HIGH_RESIDUAL_GAIN);

  // Setting gain of posture task for torque control mode
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(4.0);
  ctl.compPostureTask->weight(1);

  ctl.compEETask->reset();
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->positionTask->stiffness(30);
  ctl.compEETask->positionTask->position(ctl.taskPosition_);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(30);
  ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
  ctl.solver().addTask(ctl.compEETask);

  ctl.compPostureTask->makeCompliant(true);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  // ctl.compPostureTask->target(ctl.robot().posture());
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
  if (ctl.datastore().has("mc_kortex::setLambda")) {
    ctl.datastore().call<void, std::vector<double>>(
        "mc_kortex::setLambda", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    ctl.datastore().call<void, double>("mc_kortex::setVelThreshold", 0.1);
    ctl.datastore().call<void, double>("mc_kortex::setAccThreshold", 1);
  }

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.posTorqueFlag = false; // false: position control, true: torque control
  ctl.game.setControlMode(2);
  mc_rtc::log::success("[MonodzukuriKinovaDemo] Compliance mode initialized");
}

bool MonodzukuriKinovaDemo_Compliance::run(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // mc_rtc::log::info("[Compliance mode] changeModeAvailable: {},
  // changeModeRequest: {}", ctl.changeModeAvailable, ctl.changeModeRequest);
  if (ctl.changeModeRequest) {
    transitionTime_ += ctl.dt_ctrl;
    if (!transitionStarted_) {
      // ctl.compPostureTask->setGains(10.0, 20.0);
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      transitionStarted_ = true;
    }
    if (transitionTime_ > transitionDuration_) {
      output("OK");
      return true;
    }
  }

  // Change mode from torque to position
  if (changeModeRequest_) {
    if (ctl.robot().tvmRobot().alpha()->value().norm() < 0.01) {
      changeModeRequest_ = false;
      setPositionControl(ctl);
    }
  }

  if (!transitionStarted_ && !changeModeRequest_) {
    controlModeManager(ctl);
  }
  return false;
}

void MonodzukuriKinovaDemo_Compliance::teardown(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

void MonodzukuriKinovaDemo_Compliance::controlModeManager(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  if (ctl.posTorqueFlag != isTorqueControl_) {
    isTorqueControl_ = !isTorqueControl_;
    if (isTorqueControl_) {
      mc_rtc::log::info("[Compliance mode] Transition to torque control");
      setTorqueControl(ctl);
    } else {
      // Transition to position control
      mc_rtc::log::info("[Compliance mode] Transition to position control");
      changeModeRequest_ = true;
      ctl.compPostureTask->setGains(10.0, 20.0);
    }
  }
}

void MonodzukuriKinovaDemo_Compliance::setTorqueControl(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(4.0);
  ctl.compPostureTask->weight(1);

  ctl.compEETask->reset();
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->positionTask->stiffness(30);
  ctl.compEETask->positionTask->position(ctl.taskPosition_);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(30);
  ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
  ctl.compEETask->makeCompliant(true);
  ctl.solver().addTask(ctl.compEETask);

  ctl.compPostureTask->makeCompliant(true);

  ctl.datastore().assign<std::string>("ControlMode", "Torque");
}

void MonodzukuriKinovaDemo_Compliance::setPositionControl(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_Compliance",
                    MonodzukuriKinovaDemo_Compliance)
