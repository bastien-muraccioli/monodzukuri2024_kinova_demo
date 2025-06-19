#include "MonodzukuriKinovaDemo_NSCompliant.h"
#include "../MonodzukuriKinovaDemo.h"
#include <mc_rtc/gui/Label.h>

void MonodzukuriKinovaDemo_NSCompliant::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_NSCompliant::start(
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

  realRobot = &ctl.realRobot();

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureTarget);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.crossButtonFlag =
      false; // true: activate Dual compliance mode, false: deactivate
  ctl.triangleButtonFlag =
      true; // true: nullspace compliant, false: not compliant
  ctl.squareButtonFlag =
      false; // true: end-effector compliant, false: not compliant
  ctl.circleButtonFlag = false; // false: position control, true: torque control

  // Update the UI
  ctl.game.setControlMode(4);
  ctl.wayPoints.clear();
  ctl.kinestheticTeachingHasBeenPlayed_ = false;

  // Add GUI and log
  tool_frame = ctl.tool_frame;
  compEETask = ctl.compEETask;
  addGui(ctl);
  addLog(ctl);

  mc_rtc::log::success(
      "[MonodzukuriKinovaDemo] Null Space Compliant mode initialized");
}

bool MonodzukuriKinovaDemo_NSCompliant::run(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  t += ctl.timeStep;

  // Exit State
  if (ctl.changeModeRequest) {
    transitionTime_ += ctl.dt_ctrl;
    if (!transitionStarted_) {
      ctl.compPostureTask->reset();
      ctl.compPostureTask->refVel(Eigen::VectorXd::Zero(ctl.jointNumber));
      ctl.compPostureTask->setGains(10.0, 20.0);
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      transitionStarted_ = true;
    }
    if (transitionTime_ > transitionDuration_) {
      output("OK");
      return true;
    }
  }

  // Transition to dual compliance state
  if (ctl.crossButtonFlag) {
    // dualComplianceLoop(ctl);
    output("DC");
    return true;
  }

  // Initial state
  if (ctl.compPostureTask->eval().norm() < 0.05 && !start_moving_ &&
      !transitionStarted_) {
    mc_rtc::log::info("[Null Space mode] Start moving");
    start_moving_ = true;
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
    if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
      ctl.datastore().call("EF_Estimator::toggleActive");
    }
    nullSpaceControl(ctl);
  }

  // Change mode from torque to position
  if (start_moving_ && changeToPosCtlRequest_) {
    if (ctl.robot().tvmRobot().alpha()->value().norm() < 0.01) {
      changeToPosCtlRequest_ = false;
      setPositionControl(ctl);
    }
  }

  // While the state is running
  if (start_moving_ && !transitionStarted_ && !changeToPosCtlRequest_) {
    controlModeManager(ctl);
  }

  return false;
}

void MonodzukuriKinovaDemo_NSCompliant::teardown(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.gui()->removePlot("EE error");
}

void MonodzukuriKinovaDemo_NSCompliant::controlModeManager(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Transition to position control if requested
  if (ctl.circleButtonFlag && !isPositionControl_) {
    mc_rtc::log::info("[Null Space mode] Position control");
    isPositionControl_ = true;
    changeToPosCtlRequest_ = true;
    ctl.compPostureTask->setGains(10.0, 20.0);
  }

  // Transition to torque control if requested <=> set Null Space control
  else if (!ctl.circleButtonFlag && isPositionControl_) {
    mc_rtc::log::info("[Null Space mode] Torque control");
    isPositionControl_ = false;
    nullSpaceControl(ctl);
  }

  if (!isPositionControl_) {
    if (ctl.triangleButtonFlag && !nsCompliantFlag_) {
      mc_rtc::log::info("[Null Space mode] Nullspace compliance activated");
      nsCompliantFlag_ = true;
      ctl.compPostureTask->makeCompliant(true);
    }

    else if (nsCompliantFlag_ && !ctl.triangleButtonFlag) {
      mc_rtc::log::info("[Null Space mode] Nullspace compliance deactivated");
      nsCompliantFlag_ = false;
      ctl.compPostureTask->makeCompliant(false);
    }

    if (ctl.squareButtonFlag && !eeCompliantFlag_) {
      mc_rtc::log::info("[Null Space mode] End-effector compliance activated");
      eeCompliantFlag_ = true;
      ctl.compEETask->makeCompliant(true);
    }

    else if (eeCompliantFlag_ && !ctl.squareButtonFlag) {
      mc_rtc::log::info(
          "[Null Space mode] End-effector compliance deactivated");
      eeCompliantFlag_ = false;
      ctl.compEETask->makeCompliant(false);
    }
  }
}

void MonodzukuriKinovaDemo_NSCompliant::nullSpaceControl(
    mc_control::fsm::Controller &ctl_) {
  mc_rtc::log::info("[Null Space mode] Null Space control");
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Enable torque control and feedback from external forces estimator
  ctl.datastore().assign<std::string>("ControlMode", "Torque");
  if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(2.0);
  ctl.compPostureTask->weight(1);
  ctl.compPostureTask->makeCompliant(nsCompliantFlag_);

  ctl.compEETask->reset();
  ctl.compEETask->positionTask->reset();
  ctl.compEETask->positionTask->stiffness(400);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->reset();
  ctl.compEETask->orientationTask->stiffness(400);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->makeCompliant(eeCompliantFlag_);
  ctl.solver().addTask(ctl.compEETask);
}

void MonodzukuriKinovaDemo_NSCompliant::setPositionControl(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Disable feedback from external forces estimator (safer)
  if (ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    ctl.datastore().call("EF_Estimator::toggleActive");
  }
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  // ctl.solver().removeTask(ctl.compEETask);
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->makeCompliant(false);
  ctl.compEETask->makeCompliant(false);
}

void MonodzukuriKinovaDemo_NSCompliant::addGui(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // auto gui = ctl.gui();

  // ctl.gui()->addElement(
  //     {"Controller"},
  //     mc_rtc::gui::ArrayInput(
  //         "Error", {"x", "y", "z"},
  //         [this]() -> Eigen::Vector3d {
  //           return (compEETask->positionTask->position() -
  //                   realRobot->bodyPosW(tool_frame).translation()) *
  //                  1e3;
  //         },
  //         [this](Eigen::Vector3d v) {}));

  // ctl.gui()->addElement(
  //     {"Controller"},
  //     mc_rtc::gui::NumberInput(
  //         "Big rotor inertia",
  //         [this]() {
  //           return realRobot->mb()
  //               .joint(realRobot->mb().jointIndexByName("joint_1"))
  //               .rotorInertia();
  //         },
  //         [this](double Ir) {
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_1"), Ir);
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_2"), Ir);
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_3"), Ir);
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_4"), Ir);
  //         }),
  //     mc_rtc::gui::NumberInput(
  //         "Small rotor inertia",
  //         [this]() {
  //           return realRobot->mb()
  //               .joint(realRobot->mb().jointIndexByName("joint_5"))
  //               .rotorInertia();
  //         },
  //         [this](double Ir) {
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_5"), Ir);
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_6"), Ir);
  //           realRobot->mb().setJointRotorInertia(
  //               realRobot->mb().jointIndexByName("joint_7"), Ir);
  //         }));

  // ctl.gui()->addElement(
  //     this, {"Controller"},
  //     mc_rtc::gui::Button("Recreate plots", [this, gui]() {
  //       gui->removePlot("EE error");
  //       gui->addPlot(
  //           "EE error", mc_rtc::gui::plot::X("t", [this]() { return t; }),
  //           mc_rtc::gui::plot::Y(
  //               "t",
  //               [this]() {
  //                 return (compEETask->positionTask->position().z() -
  //                         realRobot->bodyPosW(tool_frame).translation().z())
  //                         *
  //                        1e3;
  //               },
  //               mc_rtc::gui::Color::Red));
  //     }));

  // ctl.gui()->addPlot(
  //     "EE error", mc_rtc::gui::plot::X("t", [this]() { return t; }),
  //     mc_rtc::gui::plot::Y(
  //         "t",
  //         [this]() {
  //           return (compEETask->positionTask->position().z() -
  //                   realRobot->bodyPosW(tool_frame).translation().z()) *
  //                  1e3;
  //         },
  //         mc_rtc::gui::Color::Red));
}

void MonodzukuriKinovaDemo_NSCompliant::addLog(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  //  ctl.logger().addLogEntry("realRobot_error", this, [this]() {
  //   return (compEETask->positionTask->position().z() -
  //           realRobot->bodyPosW(tool_frame).translation().z()) *
  //          1e3;
  // });
  // ctl.logger().addLogEntry("realRobot_body_vel_w_DS4_tool", this,
  //                          [this]() {
  //                            return realRobot->bodyVelW(tool_frame).linear();
  //                          });
  // ctl.logger().addLogEntry(
  //     "realRobot_body_pos_w_DS4_tool", this, [this]() {
  //       return realRobot->bodyPosW(tool_frame).translation();
  //     });
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_NSCompliant",
                    MonodzukuriKinovaDemo_NSCompliant)
