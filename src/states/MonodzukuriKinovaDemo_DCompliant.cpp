#include "MonodzukuriKinovaDemo_DCompliant.h"
#include "../MonodzukuriKinovaDemo.h"
#include <mc_rtc/gui/Label.h>

void MonodzukuriKinovaDemo_DCompliant::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_DCompliant::start(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  
  // Update the UI
  ctl.game.setControlMode(1);

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

  ctl.gui()->addElement(this, {"Controller"},
                        mc_rtc::gui::Label("Current force: ",
                                           [this]() { return currentForce_; }));
  ctl.gui()->addElement(this,{"Controller"},
                        mc_rtc::gui::NumberInput(
                            "Dual Compliance Wrench Max Threshold",
                            [this]() { return dualComplianceMaxThreshold_; },
                            [this](double threshold) {
                              dualComplianceMaxThreshold_ = threshold;
                            }));
  ctl.gui()->addElement(this,{"Controller"},
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
      transitionStarted_ = true;
    }
    if (transitionTime_ > transitionDuration_) {
      std::string outputStr = "OK";
      if(nsComplianceStateFlag_) outputStr = "NS";
      output(outputStr);
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
  ctl.gui()->removeElements(this);
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

  if(ctl.circleButtonFlag != removeWayPointFlag_){
    // mc_rtc::log::info("Removing last waypoint");
    removeWayPointFlag_ = !removeWayPointFlag_;
  }
  if(removeWayPointFlag_ != removeWayPointFlagLast_){
    removeWayPoint(ctl);
    removeWayPointFlagLast_ = removeWayPointFlag_;
  }

  if(ctl.squareButtonFlag != addWayPointFlag_){
    // mc_rtc::log::info("Adding waypoint");
    addWayPointFlag_ = !addWayPointFlag_;
  }
  if(addWayPointFlag_ != addWayPointFlagLast_)
  {
    addWayPoint(ctl);
    addWayPointFlagLast_ = addWayPointFlag_;
  }

}

void MonodzukuriKinovaDemo_DCompliant::dualComplianceControl(
    mc_control::fsm::Controller &ctl_) {
  // mc_rtc::log::info("[Null Space mode] DualCompliance Loop control");
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // ctl.compPostureTask->reset();
  ctl.compEETask->reset();
}

void MonodzukuriKinovaDemo_DCompliant::addWayPoint(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  auto &robot = ctl.robot(ctl.robots()[0].name());
  auto &rjo = robot.refJointOrder();

  if(ctl.kinestheticTeachingHasBeenPlayed_){
    for (size_t i = 0; i < ctl.wayPoints.size(); ++i) {
    }
    ctl.wayPoints.clear();
    ctl.kinestheticTeachingHasBeenPlayed_ = false;
  }

  std::map<std::string, std::vector<double>> currentPosture;
  auto jointNames = robot.refJointOrder();
  const auto &q_tricked = robot.mbc().q;
  const auto &currentTarget = ctl.compPostureTask->posture();

  std::vector<double> q_current_short(ctl.jointNumber);
  std::vector<double> q_target_short(ctl.jointNumber);

  for (size_t i = 0; i < ctl.jointNumber; ++i) {
    const auto &jointName = rjo[i];
    int idx = robot.jointIndexByName(jointName);
    q_current_short[i] = q_tricked[idx][0];
    q_target_short[i] = currentTarget[idx][0];
  }

  std::vector<double> correctedAngles = computeAngleOffsets(ctl.jointNumber, q_target_short, q_current_short);

  for (size_t i = 0; i < ctl.jointNumber; ++i) {
    const auto &jointName = rjo[i];
    if (robot.hasJoint(jointName)) {
      double q_corrected_short = q_current_short[i] - correctedAngles[i];
      mc_rtc::log::info("Adding joint {} with value {}", jointName, q_corrected_short);
      currentPosture[jointName] = {q_corrected_short};
    }
  }

  sva::PTransformd posEE = robot.bodyPosW(ctl.tool_frame);
  // mc_rtc::log::info("Adding waypoint with current posture: {}", currentPosture);
  ctl.wayPoints.emplace_back(currentPosture, posEE);
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

std::vector<double> MonodzukuriKinovaDemo_DCompliant::computeAngleOffsets(
    size_t actuator_count,
    const std::vector<double> &target_angles,
    const std::vector<double> &current_angles)
{
  if (target_angles.size()  != actuator_count ||
      current_angles.size() != actuator_count)
  {
    throw std::invalid_argument(
      "computeAngleOffsets: size of angle arrays must equal actuator_count");
  }

  std::vector<double> offsets(actuator_count, 0.0);

  for (size_t i = 0; i < actuator_count; ++i)
  {
    double tgt = target_angles[i];
    double cur = current_angles[i];

    // If current is > (target + π), subtract 2π to wrap it down.
    if (cur > tgt + M_PI)
    {
      offsets[i] = -2.0 * M_PI;
    }
    // If current is < (target - π), add 2π to wrap it up.
    else if (cur < tgt - M_PI)
    {
      offsets[i] = +2.0 * M_PI;
    }
    // Otherwise, no offset is needed (current is within ±π of target).
    else
    {
      offsets[i] = 0.0;
    }
  }

  return offsets;
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_DCompliant",
                    MonodzukuriKinovaDemo_DCompliant)
