#include "MonodzukuriKinovaDemo_DCompliant.h"
#include "../MonodzukuriKinovaDemo.h"
#include <mc_rtc/gui/Label.h>
#include <mc_rtc/gui/Point3D.h>

void MonodzukuriKinovaDemo_DCompliant::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_DCompliant::start(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  
  // Update the UI
  ctl.game.setControlMode(4);

  // Enable torque control and feedback from external forces estimator
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
    if (!ctl.datastore().call<bool>("EF_Estimator::isActive")) {
      ctl.datastore().call("EF_Estimator::toggleActive");
    }

  // Enable force sensor usage if not active
  if (!ctl.datastore().call<bool>("EF_Estimator::useForceSensor")) {
    ctl.datastore().call("EF_Estimator::toggleForceSensor");
  }

  ctl.datastore().call<void, double>("EF_Estimator::setGain",
                                     HIGH_RESIDUAL_GAIN);


  // Set the shoulder task
  // ctl.compShoulderTask->reset();
  // ctl.compShoulderTask->positionTask->stiffness(0);
  // ctl.compShoulderTask->positionTask->damping(0.0);
  // ctl.compShoulderTask->positionTask->weight(0);
  // ctl.compShoulderTask->orientationTask->stiffness(0);
  // ctl.compShoulderTask->orientationTask->damping(0.0);
  // ctl.compShoulderTask->orientationTask->weight(0);
  // ctl.compShoulderTask->makeCompliant(true);
  // ctl.solver().addTask(ctl.compShoulderTask);

  // Set the end-effector task
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(400);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(400);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->makeCompliant(false);
  ctl.solver().addTask(ctl.compEETask);

  // Set the posture task
  ctl.compPostureTask->reset();
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(2.0);
  ctl.compPostureTask->makeCompliant(true);

  ctl.changeModeAvailable = true;
  ctl.changeModeRequest = false;
  ctl.crossButtonFlag = false; // go back to null space compliant mode
  ctl.triangleButtonFlag = false; //run points
  ctl.squareButtonFlag = false; // add a point
  ctl.circleButtonFlag = false; // Remove a point

  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::Label("Current force: ",
                                           [this]() { return currentForce_; }));
  ctl.gui()->addElement({"Controller"},
                        mc_rtc::gui::NumberInput(
                            "Dual Compliance Wrench Max Threshold",
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

  mc_rtc::log::success("[MonodzukuriKinovaDemo] Dual Compliant mode initialized");
}

bool MonodzukuriKinovaDemo_DCompliant::run(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // mc_rtc::log::info("[MonodzukuriKinovaDemo_DCompliant] Running Dual Compliant mode");

  if(ctl.crossButtonFlag && !nsComplianceStateFlag_) nsComplianceStateFlag_ = true;

  // Exit State
  if (ctl.changeModeRequest || nsComplianceStateFlag_) {
    transitionTime_ += ctl.dt_ctrl;
    if (!transitionStarted_) {
      ctl.compPostureTask->reset();
      ctl.compPostureTask->refVel(Eigen::VectorXd::Zero(ctl.jointNumber));
      ctl.compPostureTask->setGains(10.0, 20.0);
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      ctl.compEETask->orientationTask->refVel(Eigen::Vector3d(0, 0, 0));
      // ctl.compShoulderTask->reset();
      // ctl.compShoulderTask->positionTask->refVel(Eigen::Vector3d(0, 0, 0));
      // ctl.compShoulderTask->orientationTask->refVel(Eigen::Vector3d(0, 0, 0));
      transitionStarted_ = true;
    }
    if (transitionTime_ > transitionDuration_) {
      std::string outputStr = "OK";
      if(nsComplianceStateFlag_) outputStr = "NS";
      output(outputStr);
      // ctl.solver().removeTask(ctl.compShoulderTask);
      ctl.wayPoints.clear();
      return true;
    }
  }

  // Run the waypoint control state if requested
  if(ctl.triangleButtonFlag && !runWayPointFlag_)
  {
    mc_rtc::log::info("Running waypoint control state");
    runWayPointFlag_ = true;
    output("RUN");
    return true;
  }

  // While the state is running
  if (!transitionStarted_ && !changeModeRequest_ && !nsComplianceStateFlag_) {
    controlModeManager(ctl);
  }

  return false;
}

void MonodzukuriKinovaDemo_DCompliant::teardown(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
}

void MonodzukuriKinovaDemo_DCompliant::controlModeManager(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  currentForce_ =
      ctl.robot().forceSensor("EEForceSensor").wrench().vector().norm();

  if(dualComplianceLoopFlag_){
    dualComplianceControl(ctl); // Reset the tasks position and orientation
  }

  if (currentForce_ >= dualComplianceMaxThreshold_ && !dualComplianceLoopFlag_) {

      // mc_rtc::log::info("Above threshold, compliant end-effector control activated");
      dualComplianceLoopFlag_ = true;
      ctl.compEETask->reset();
      ctl.compEETask->positionTask->stiffness(0);
      ctl.compEETask->positionTask->damping(15.0);
      ctl.compEETask->orientationTask->stiffness(0);
      ctl.compEETask->orientationTask->damping(15.0);
      ctl.compEETask->makeCompliant(true);
  }
  else if (currentForce_ < dualComplianceMinThreshold_ &&
             dualComplianceLoopFlag_) {
    // mc_rtc::log::info("Below threshold, compliant end-effector control deactivated");
    dualComplianceLoopFlag_ = false;
    ctl.compEETask->reset();
    ctl.compEETask->positionTask->stiffness(400);
    ctl.compEETask->positionTask->weight(10000);
    ctl.compEETask->orientationTask->stiffness(400);
    ctl.compEETask->orientationTask->weight(10000);
    ctl.compEETask->makeCompliant(false);
  }

  if(ctl.circleButtonFlag && !removeWayPointFlag_){
    // mc_rtc::log::info("Removing last waypoint");
    removeWayPointFlag_ = true;
    removeWayPoint(ctl);
  }
  if(!ctl.circleButtonFlag && removeWayPointFlag_){
    removeWayPointFlag_ = false;
  }
  if(ctl.squareButtonFlag && !addWayPointFlag_){
    // mc_rtc::log::info("Adding waypoint");
    addWayPointFlag_ = true;
    addWayPoint(ctl);
  }
  if(!ctl.squareButtonFlag && addWayPointFlag_){
    addWayPointFlag_ = false;
  }

}

void MonodzukuriKinovaDemo_DCompliant::dualComplianceControl(
    mc_control::fsm::Controller &ctl_) {
  // mc_rtc::log::info("[Null Space mode] DualCompliance Loop control");
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.compPostureTask->reset();
  ctl.compEETask->reset();
  // ctl.compShoulderTask->reset();
}

void MonodzukuriKinovaDemo_DCompliant::addWayPoint(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  auto & robot = ctl.robot(ctl.robots()[0].name());
  auto & rjo = robot.refJointOrder();

  std::map<std::string, std::vector<double>> currentPosture;
  auto jointNames = robot.refJointOrder();
  auto q = robot.mbc().q;
  mc_rtc::log::info("jointNames: {}", jointNames[0]);
  // Construct the current posture map
  for (const auto &jointName : jointNames) {
    if (robot.hasJoint(jointName)) {
      mc_rtc::log::info("Adding joint {} with value {}", jointName, q[robot.jointIndexByName(jointName)][0]);
      currentPosture[jointName] = {q[robot.jointIndexByName(jointName)][0]};
    }
  }
  
  sva::PTransformd posEE = robot.bodyPosW(ctl.tool_frame);
  // sva::PTransformd posShoulder = robot.bodyPosW(ctl.shoulder_frame);
  ctl.wayPoints.emplace_back(currentPosture, posEE);
  // ctl.wayPoints.emplace_back(posEE, posShoulder);

  // mc_rtc::log::info("Added waypoint:\n\tEE Position: {}, Orientation: {}\n\tShoulder Position: {}, Orientation: {}",
  //                   posEE.translation().transpose(),
  //                   posEE.rotation().transpose(),
  //                   posShoulder.translation().transpose(),
  //                   posShoulder.rotation().transpose());
}


void MonodzukuriKinovaDemo_DCompliant::removeWayPoint(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  if (!ctl.wayPoints.empty()) {
    ctl.wayPoints.pop_back();
    mc_rtc::log::info("Removed last waypoint. Remaining waypoints: {}", ctl.wayPoints.size());
  } else {
    mc_rtc::log::warning("No waypoints to remove.");
  }
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_DCompliant",
                    MonodzukuriKinovaDemo_DCompliant)
