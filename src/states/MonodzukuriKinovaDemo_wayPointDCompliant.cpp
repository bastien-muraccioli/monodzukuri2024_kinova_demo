#include "MonodzukuriKinovaDemo_wayPointDCompliant.h"
#include "../MonodzukuriKinovaDemo.h"
#include <mc_rtc/gui/Label.h>

void MonodzukuriKinovaDemo_wayPointDCompliant::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_wayPointDCompliant::start(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  mc_rtc::log::info(
          "[MonodzukuriKinovaDemo] Waypoint D Compliant mode initialized");
  // ctl.compShoulderTask->reset();
  // ctl.compShoulderTask->positionTask->stiffness(10);
  // ctl.compShoulderTask->positionTask->damping(30.0);
  // ctl.compShoulderTask->positionTask->weight(1000);
  // ctl.compShoulderTask->orientationTask->stiffness(10);
  // ctl.compShoulderTask->orientationTask->damping(30.0);
  // ctl.compShoulderTask->orientationTask->weight(1000);
  // ctl.compShoulderTask->makeCompliant(true);
  ctl.compPostureTask->reset();
  // ctl.compPostureTask->stiffness(30);
  // ctl.compPostureTask->damping(30.0);
  ctl.compPostureTask->stiffness(10);
  // ctl.compPostureTask->damping(10);
  ctl.compPostureTask->weight(10000);
  ctl.compPostureTask->makeCompliant(true);
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(0);
  // ctl.compEETask->positionTask->damping(30.0);
  ctl.compEETask->positionTask->weight(0);
  ctl.compEETask->orientationTask->stiffness(0);
  // ctl.compEETask->orientationTask->damping(30.0);
  ctl.compEETask->orientationTask->weight(0);
  // ctl.compEETask->makeCompliant(true);

  // Set the first waypoint as target
  if (!ctl.wayPoints.empty()) {
      ctl.compPostureTask->target(ctl.wayPoints[wayPointIndex_]);
    // ctl.compEETask->set_ef_pose(ctl.wayPoints[wayPointIndex_]);
    // ctl.compShoulderTask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
    wayPointIndex_++;
  } else {
    mc_rtc::log::warning("No waypoints set, please add waypoints before running the controller.");
  }

}

bool MonodzukuriKinovaDemo_wayPointDCompliant::run(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Check if ctl.wayPoints is empty
  if (ctl.wayPoints.empty()) {
    output("OK");
    return true;
  }

      mc_rtc::log::info("Current Distance to target: {}",
                      ctl.compPostureTask->eval().norm());

  // if the target was reached, move to the next waypoint
  // if(ctl.compEETask->eval().norm() < 0.01 && ctl.compShoulderTask->eval().norm() < 0.01)
  if(ctl.compPostureTask->eval().norm() < 0.1)
  {
    mc_rtc::log::info("Reached waypoint {} of {}", wayPointIndex_, ctl.wayPoints.size());

    // mc_rtc::log::info("Current Distance to shoulder target: {}",
    //                   ctl.compShoulderTask->eval().norm());
    if(wayPointIndex_ < ctl.wayPoints.size())
    {
      // ctl.compEETask->set_ef_pose(ctl.wayPoints[wayPointIndex_].first);
      // ctl.compShoulderTask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
      ctl.compPostureTask->target(ctl.wayPoints[wayPointIndex_]);
      // ctl.compPostureTask->refVel(Eigen::VectorXd::Ones(ctl.wayPoints[wayPointIndex_].size()) * 0.1);
      wayPointIndex_++;
    }
    else
    {
      mc_rtc::log::info("All waypoints reached.");
      output("OK");
      return true;
    }
  }

  return false;
}

void MonodzukuriKinovaDemo_wayPointDCompliant::teardown(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  // ctl.solver().removeTask(ctl.compShoulderTask);
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(400);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(400);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compPostureTask->weight(1);
  ctl.wayPoints.clear();
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_wayPointDCompliant",
                    MonodzukuriKinovaDemo_wayPointDCompliant)
