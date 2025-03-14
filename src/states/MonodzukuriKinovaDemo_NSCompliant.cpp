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

  // admittance_task = std::make_shared<mc_tasks::force::AdmittanceTask>(
  //     "DS4_tool", ctl.robots(), ctl.robot().robotIndex(), 5.0, 10000.0);

  realRobot = &ctl.realRobot();

  t = 0;

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->target(ctl.postureTarget);
  ctl.compPostureTask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);
  ctl.datastore().assign<std::string>("ControlMode", "Position");

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.activateFlag = false;
  ctl.nsCompliantFlag = true;
  ctl.compliantFlag = false;
  ctl.posTorqueFlag = false; // false: position control, true: torque control

  auto gui = ctl.gui();

  compEETask = ctl.compEETask;

  ctl.game.setControlMode(4);
  if (ctl.datastore().has("mc_kortex::setLambda")) {
    ctl.datastore().call<void, std::vector<double>>(
        "mc_kortex::setLambda", {5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0});
    ctl.datastore().call<void, double>("mc_kortex::setVelThreshold", 0.1);
    ctl.datastore().call<void, double>("mc_kortex::setAccThreshold", 1);
  }

  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::NumberInput(
                            "Dual Compliance Wrench Threshold",
                            [this]() { return dualComplianceMaxThreshold_; },
                            [this](double threshold) {
                              dualComplianceMaxThreshold_ = threshold;
                            }));
  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::Label("Current force: ",
                                           [this]() { return currentForce_; }));
  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::NumberInput(
                            "Dual Compliance Wrench Threshold",
                            [this]() { return dualComplianceMaxThreshold_; },
                            [this](double threshold) {
                              dualComplianceMaxThreshold_ = threshold;
                            }));
  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::NumberInput(
                            "Dual Compliance Wrench Min Threshold",
                            [this]() { return dualComplianceMinThreshold_; },
                            [this](double threshold) {
                              dualComplianceMinThreshold_ = threshold;
                            }));
  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::Label("Current force: ",
                                           [this]() { return currentForce_; }));

  ctl.gui()->addElement(
      {"Controller"},
      mc_rtc::gui::ArrayInput(
          "Error", {"x", "y", "z"},
          [this]() -> Eigen::Vector3d {
            return (compEETask->positionTask->position() -
                    realRobot->bodyPosW("DS4_tool").translation()) *
                   1e3;
          },
          [this](Eigen::Vector3d v) {}));

  ctl.gui()->addElement(
      {"Controller"},
      mc_rtc::gui::NumberInput(
          "Big rotor inertia",
          [this]() {
            return realRobot->mb()
                .joint(realRobot->mb().jointIndexByName("joint_1"))
                .rotorInertia();
          },
          [this](double Ir) {
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_1"), Ir);
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_2"), Ir);
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_3"), Ir);
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_4"), Ir);
          }),
      mc_rtc::gui::NumberInput(
          "Small rotor inertia",
          [this]() {
            return realRobot->mb()
                .joint(realRobot->mb().jointIndexByName("joint_5"))
                .rotorInertia();
          },
          [this](double Ir) {
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_5"), Ir);
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_6"), Ir);
            realRobot->mb().setJointRotorInertia(
                realRobot->mb().jointIndexByName("joint_7"), Ir);
          }));

  ctl.gui()->addElement(
      this, {"Controller"},
      mc_rtc::gui::Button("Recreate plots", [this, gui]() {
        gui->removePlot("EE error");
        gui->addPlot(
            "EE error", mc_rtc::gui::plot::X("t", [this]() { return t; }),
            mc_rtc::gui::plot::Y(
                "t",
                [this]() {
                  return (compEETask->positionTask->position().z() -
                          realRobot->bodyPosW("DS4_tool").translation().z()) *
                         1e3;
                },
                mc_rtc::gui::Color::Red));
      }));

  ctl.gui()->addPlot(
      "EE error", mc_rtc::gui::plot::X("t", [this]() { return t; }),
      mc_rtc::gui::plot::Y(
          "t",
          [this]() {
            return (compEETask->positionTask->position().z() -
                    realRobot->bodyPosW("DS4_tool").translation().z()) *
                   1e3;
          },
          mc_rtc::gui::Color::Red));

  ctl.logger().addLogEntry("realRobot_error", this, [this]() {
    return (compEETask->positionTask->position().z() -
            realRobot->bodyPosW("DS4_tool").translation().z()) *
           1e3;
  });
  ctl.logger().addLogEntry("realRobot_body_vel_w_DS4_tool", this, [this]() {
    return realRobot->bodyVelW("DS4_tool").linear();
  });
  ctl.logger().addLogEntry("realRobot_body_pos_w_DS4_tool", this, [this]() {
    return realRobot->bodyPosW("DS4_tool").translation();
  });

  mc_rtc::log::success("[MonodzukuriKinovaDemo] Switched to Sensor Testing "
                       "state - Position controlled");
}

bool MonodzukuriKinovaDemo_NSCompliant::run(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  t += ctl.timeStep;

  // Exit State
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
  if (start_moving_ && changeModeRequest_) {
    if (ctl.robot().tvmRobot().alpha()->value().norm() < 0.01) {
      changeModeRequest_ = false;
      setPositionControl(ctl);
    }
  }

  // While the state is running
  if (start_moving_ && !transitionStarted_ && !changeModeRequest_) {
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

  if (ctl.posTorqueFlag && !isPositionControl_) {
    mc_rtc::log::info("[Null Space mode] Position control");
    isPositionControl_ = true;
    changeModeRequest_ = true;
    ctl.compPostureTask->setGains(10.0, 20.0);
  } else if (!ctl.posTorqueFlag && isPositionControl_) {
    mc_rtc::log::info("[Null Space mode] Torque control");
    isPositionControl_ = false;
    if (dualComplianceFlag_) {
      dualComplianceLoop(ctl);
    } else {
      nullSpaceControl(ctl);
    }
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
  }

  if (!isPositionControl_) {
    // If press the X button, activate/deactivate the dual compliance
    if (ctl.activateFlag && !dualComplianceFlag_) {
      mc_rtc::log::info("[Null Space mode] Dual Compliance activated");
      dualComplianceFlag_ = true;
      ctl.game.setControlMode(1);
    } else if (dualComplianceFlag_ && !ctl.activateFlag) {
      mc_rtc::log::info("[Null Space mode] Dual Compliance deactivated");
      dualComplianceFlag_ = false;
      ctl.game.setControlMode(4);
      nullSpaceControl(ctl);
    }
    if (dualComplianceFlag_) {
      dualComplianceLoop(ctl);
    }

    if (ctl.nsCompliantFlag && !nsCompliantFlag_) {
      mc_rtc::log::info("[Null Space mode] Nullspace compliance activated");
      nsCompliantFlag_ = true;
      ctl.compPostureTask->makeCompliant(true);
    } else if (nsCompliantFlag_ && !ctl.nsCompliantFlag) {
      mc_rtc::log::info("[Null Space mode] Nullspace compliance deactivated");
      nsCompliantFlag_ = false;
      ctl.compPostureTask->makeCompliant(false);
    }

    if (ctl.compliantFlag && !eeCompliantFlag_) {
      mc_rtc::log::info("[Null Space mode] End-effector compliance activated");
      eeCompliantFlag_ = true;
      ctl.compEETask->makeCompliant(true);
    } else if (eeCompliantFlag_ && !ctl.compliantFlag) {
      mc_rtc::log::info(
          "[Null Space mode] End-effector compliance deactivated");
      eeCompliantFlag_ = false;
      ctl.compEETask->makeCompliant(false);
    }
  }
}

void MonodzukuriKinovaDemo_NSCompliant::dualComplianceLoop(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Activate the admittance control if the force is below the threshold
  // currentForce_ = ctl.robot().forceSensor("EEForceSensor").force().norm();
  currentForce_ =
      ctl.robot().forceSensor("EEForceSensor").wrench().vector().norm();
  // currentForce_ =
  // ctl.robot().forceSensor("EEForceSensor").FT_sensor_wrench().norm();
  mc_rtc::log::info("[Null Space mode] Current force: {}", currentForce_);

  if (currentForce_ >= dualComplianceMaxThreshold_) {

    mc_rtc::log::info(
        "Above threshold, compliant end-effector control activated");

    // IMPORTANT: disable feedback from external forces estimator, not
    // compatible with admittance control if
    // (ctl.datastore().call<bool>("EF_Estimator::isActive")) {
    //   ctl.datastore().call("EF_Estimator::toggleActive");
    // }
    dualComplianceControl(ctl);
    if (!dualComplianceLoopFlag_) {
      mc_rtc::log::info("DualCompliance control activated");
      dualComplianceLoopFlag_ = true;
      ctl.compPostureTask->stiffness(0.0);
      ctl.compPostureTask->damping(2.0);
      ctl.compPostureTask->weight(1);
      ctl.compPostureTask->makeCompliant(nsCompliantFlag_);
      ctl.compEETask->positionTask->stiffness(0);
      ctl.compEETask->positionTask->damping(15);
      ctl.compEETask->positionTask->weight(10000);
      ctl.compEETask->orientationTask->stiffness(0);
      ctl.compEETask->orientationTask->damping(15);
      ctl.compEETask->orientationTask->weight(10000);
      ctl.compEETask->makeCompliant(true);
      ctl.datastore().assign<std::string>("ControlMode", "Torque");
    }
  } else if (currentForce_ < dualComplianceMinThreshold_ &&
             dualComplianceLoopFlag_) {
    mc_rtc::log::info("Below threshold, null space control activated");
    if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
      ctl.datastore().call("EF_Estimator::toggleActive");
    }
    nullSpaceControl(ctl);
    dualComplianceLoopFlag_ = false;
  } else if (currentForce_ >= dualComplianceMinThreshold_ &&
             dualComplianceLoopFlag_) {
    // mc_rtc::log::info("Within threshold, dual compliance control activated");
    mc_rtc::log::info(
        "Within threshold, compliant end-effector control activated");
    dualComplianceControl(ctl);
  }
}

void MonodzukuriKinovaDemo_NSCompliant::dualComplianceControl(
    mc_control::fsm::Controller &ctl_) {
  mc_rtc::log::info("[Null Space mode] DualCompliance Loop control");
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // ctl.solver().removeTask(ctl.compEETask);
  ctl.compPostureTask->reset();
  ctl.compEETask->reset();

  // admittance_task->reset();
  // admittance_task->admittance(sva::ForceVecd(Eigen::Vector6d::Zero()));
  // admittance_task->weight(10000);
  // ctl.solver().addTask(admittance_task);
}

void MonodzukuriKinovaDemo_NSCompliant::nullSpaceControl(
    mc_control::fsm::Controller &ctl_) {
  mc_rtc::log::info("[Null Space mode] Null Space control");
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(1.0);
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
  // ctl.solver().removeTask(ctl.compEETask);
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.5);
  ctl.compPostureTask->makeCompliant(false);
  ctl.datastore().assign<std::string>("ControlMode", "Position");
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_NSCompliant",
                    MonodzukuriKinovaDemo_NSCompliant)
