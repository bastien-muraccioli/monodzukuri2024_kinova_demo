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

  realRobot = &ctl.realRobot();

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureTarget);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.compliantFlag = true;
  ctl.posTorqueFlag = false; // false: position control, true: torque control

  ctl.game.setControlMode(6);
  if (ctl.datastore().has("mc_kortex::setLambda")) {
    ctl.datastore().call<void, std::vector<double>>(
        "mc_kortex::setLambda", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    ctl.datastore().call<void, double>("mc_kortex::setVelThreshold", 0.1);
    ctl.datastore().call<void, double>("mc_kortex::setAccThreshold", 1);
  }

  ctl.gui()->addElement(this, {"Controller", "Sin"},
                        mc_rtc::gui::Trajectory("Trajectory", [this]() {
                          return visualSinTraj();
                        }));

  ctl.logger().addLogEntry("realRobot_body_vel_w_DS4_tool", this, [this]() {
    return realRobot->bodyVelW("DS4_tool").linear();
  });
  ctl.logger().addLogEntry("realRobot_body_pos_w_DS4_tool", this, [this]() {
    return realRobot->bodyPosW("DS4_tool").translation();
  });

  // ctl.compEETask->positionTask->reset();
  // init_x = ctl.compEETask->positionTask->position().x();
  // init_z = ctl.compEETask->positionTask->position().y();

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
    ctl.compEETask->positionTask->stiffness(400);
    ctl.compEETask->positionTask->position(ctl.taskPosition_);
    ctl.compEETask->orientationTask->weight(10000);
    ctl.compEETask->orientationTask->stiffness(400);
    ctl.compEETask->orientationTask->orientation(ctl.taskOrientation_);
    ctl.compEETask->makeCompliant(true);
    ctl.solver().addTask(ctl.compEETask);
    ctl.compPostureTask->makeCompliant(false);
    ctl.compPostureTask->stiffness(0.5);
    ctl.datastore().assign<std::string>("ControlMode", "Position");
  }

  if (start_moving_ && changeModeRequest_) {
    if (ctl.robot().tvmRobot().alpha()->value().norm() < 0.01) {
      changeModeRequest_ = false;
      setPositionControl(ctl);
    }
  }

  // Start to do sinus movement
  if (start_moving_ && !transitionStarted_ && !changeModeRequest_) {
    controlModeManager(ctl);
    ctlTime_ += ctl.dt_ctrl;
    ctl.compEETask->positionTask->position(Eigen::Vector3d(
        init_x, yValue_, init_z + R_ * std::sin(omega_ * 1 * ctlTime_)));
    ctl.compEETask->positionTask->refVel(
        Eigen::Vector3d(0, 0.1 * (yDirection_ ? 1 : -1),
                        1 * omega_ * R_ * std::cos(omega_ * 1 * ctlTime_)));
    ctl.compEETask->positionTask->refAccel(Eigen::Vector3d(
        0, 0, -1 * omega_ * omega_ * R_ * std::sin(omega_ * 1 * ctlTime_)));

    yValue_ += (yDirection_ ? 1 : -1) * ctl.dt_ctrl * 0.1;

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

std::vector<Eigen::Vector3d>
MonodzukuriKinovaDemo_SinCompliance::visualSinTraj() {
  const int num_points = 225;
  std::vector<Eigen::Vector3d> traj(num_points);
  double t = 0.0;
  double y = 0.0;
  bool dir = true;
  for (size_t i = 0; i < num_points; i++) {
    t += 0.01;
    traj[i] =
        Eigen::Vector3d(init_x, y, init_z + R_ * std::sin(omega_ * 1 * t));
    y += (dir ? 1 : -1) * 0.01 * 1 * 0.1;
    if (y >= maxY_ || y <= minY_) {
      dir = !dir;
    }
  }

  return traj;
}

void MonodzukuriKinovaDemo_SinCompliance::controlModeManager(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // If press the Circle button, change between position and torque control
  if (ctl.posTorqueFlag && !isTorqueControl_) {
    mc_rtc::log::info("[Sinus Compliance mode] Torque controlled");
    // Enable feedback from external forces estimator (safer)
    if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
      ctl.datastore().call("EF_Estimator::toggleActive");
    }
    isTorqueControl_ = true;
    ctl.compEETask->positionTask->stiffness(200);
    ctl.compEETask->orientationTask->stiffness(200);
    ctl.compEETask->makeCompliant(isCompliantControl_);
    ctl.compPostureTask->stiffness(0.0);
    ctl.compPostureTask->damping(5.0);
    ctl.compPostureTask->makeCompliant(true);
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
  } else if (isTorqueControl_ && !ctl.posTorqueFlag) {
    mc_rtc::log::info("[Sinus Compliance mode] Position controlled");
    isTorqueControl_ = false;
    changeModeRequest_ = true;
    ctl.compEETask->refAccel(Eigen::Vector6d ::Zero());
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

void MonodzukuriKinovaDemo_SinCompliance::setPositionControl(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // Disable feedback from external forces estimator (safer)
  if (ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.compEETask->positionTask->stiffness(10);
  ctl.compEETask->orientationTask->stiffness(10);
  ctl.compEETask->makeCompliant(false);
  ctl.compPostureTask->makeCompliant(false);
  ctl.compPostureTask->stiffness(0.5);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_SinCompliance",
                    MonodzukuriKinovaDemo_SinCompliance)
