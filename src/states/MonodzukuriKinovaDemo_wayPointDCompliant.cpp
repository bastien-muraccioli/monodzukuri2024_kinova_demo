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
  ctl.compPostureTask->stiffness(50.0);
  ctl.compPostureTask->damping(50.0);
  ctl.compPostureTask->weight(1000);
  ctl.compPostureTask->makeCompliant(true);
  
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(50.0);
  ctl.compEETask->positionTask->damping(20.0);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(50.0);
  ctl.compEETask->orientationTask->damping(20.0);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compEETask->makeCompliant(false);
  ctl.solver().removeTask(ctl.compEETask);

  // Set the first waypoint as target
  if (!ctl.wayPoints.empty()) {
    ctl.compPostureTask->target(ctl.wayPoints[wayPointIndex_].first);
    ctl.compEETask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
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

      mc_rtc::log::info("Current Distance to end-effector target: {}",
                      ctl.compEETask->eval().norm());

  // if the target was reached, move to the next waypoint
  // if(ctl.compEETask->eval().norm() < 0.01 && ctl.compShoulderTask->eval().norm() < 0.01)
  if(ctl.compEETask->eval().norm() < 0.1 && ctl.compPostureTask->eval().norm() < 0.1)
  {
    mc_rtc::log::info("Reached waypoint {} of {}", wayPointIndex_, ctl.wayPoints.size());

    // mc_rtc::log::info("Current Distance to shoulder target: {}",
    //                   ctl.compShoulderTask->eval().norm());
    if(wayPointIndex_ < ctl.wayPoints.size())
    {
      // ctl.compShoulderTask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
      ctl.compPostureTask->target(ctl.wayPoints[wayPointIndex_].first);
      // ctl.compEETask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
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
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(2.0);
  ctl.compPostureTask->weight(1);
  ctl.wayPoints.clear();
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_wayPointDCompliant",
                    MonodzukuriKinovaDemo_wayPointDCompliant)
