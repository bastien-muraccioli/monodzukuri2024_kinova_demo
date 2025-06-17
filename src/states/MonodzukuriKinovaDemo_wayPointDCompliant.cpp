#include "MonodzukuriKinovaDemo_wayPointDCompliant.h"
#include "../MonodzukuriKinovaDemo.h"
#include <cmath>
#include <mc_rtc/gui/Label.h>

void MonodzukuriKinovaDemo_wayPointDCompliant::configure(
    const mc_rtc::Configuration &config) {}

void MonodzukuriKinovaDemo_wayPointDCompliant::start(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  mc_rtc::log::info(
          "[MonodzukuriKinovaDemo] Waypoint D Compliant mode initialized");

  ctl.compPostureTask->reset();
  stiffness_posture_ = stiffnessMin_;
  damping_posture_ = 3*std::sqrt(stiffness_posture_);
  stiffness_task_ = stiffnessMin_/2;
  damping_task_ = 4*std::sqrt(stiffness_task_);
  ctl.compPostureTask->stiffness(stiffness_posture_);
  ctl.compPostureTask->damping(damping_posture_);
  ctl.compPostureTask->weight(1000);
  ctl.compPostureTask->makeCompliant(true);
  
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(stiffness_task_);
  ctl.compEETask->positionTask->damping(damping_task_);
  ctl.compEETask->positionTask->weight(100);
  ctl.compEETask->orientationTask->stiffness(stiffness_task_);
  ctl.compEETask->orientationTask->damping(damping_task_);
  ctl.compEETask->orientationTask->weight(100);
  ctl.compEETask->makeCompliant(true);
  taskEEInSolver_ = true;
  // ctl.solver().removeTask(ctl.compEETask);

  // Set the first waypoint as target
  if (!ctl.wayPoints.empty()) {
    ctl.compPostureTask->target(ctl.wayPoints[wayPointIndex_].first);
    ctl.compEETask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
    wayPointIndex_++;
  } else {
    mc_rtc::log::warning("[MonodzukuriKinovaDemo] No waypoints set, please add waypoints before running the controller.");
  }

  // Compute A and C for stiffness adjustment
  A_ = (-stiffnessMax_ + stiffnessMin_) / (exp(k_slope_) - 1);
  C_ = stiffnessMax_ - A_;
  addGui(ctl);
}

bool MonodzukuriKinovaDemo_wayPointDCompliant::run(mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);

  // Check if ctl.wayPoints is empty or x button is pressed
  if (ctl.wayPoints.empty() || ctl.crossButtonFlag) {
    output("OK");
    return true;
  }

  // diff_eval_posture_ = ctl.compPostureTask->speed().norm();
  // mc_rtc::log::info("Difference in posture evaluation: {}", diff_eval_posture_);

  mc_rtc::log::info("[MonodzukuriKinovaDemo] Current Distance to posture target: {}",
                    ctl.compPostureTask->eval().norm());
  mc_rtc::log::info("[MonodzukuriKinovaDemo] Current Posture Speed: {}",
                    ctl.compPostureTask->speed().norm());
  mc_rtc::log::info("[MonodzukuriKinovaDemo] Current Distance to end-effector target: {}",
                    ctl.compEETask->eval().norm());

  // Run a counter to check if the posture task is not moving
  if(ctl.compPostureTask->speed().norm() < 0.01 && !timeOut_)
  {
    counter_ += ctl.timeStep;
    if (counter_ > maxTime_) {
      mc_rtc::log::warning("[MonodzukuriKinovaDemo] Posture task has not moved for {} seconds.",
                           maxTime_);
      timeOut_ = true;
      if(taskEEInSolver_)
      {
        mc_rtc::log::info("[MonodzukuriKinovaDemo] Removing end-effector task from solver");
        ctl.solver().removeTask(ctl.compEETask);
        taskEEInSolver_ = false;
      }
    }
  }

  if(ctl.compPostureTask->eval().norm() < 1.0)
  {
    stiffness_posture_ = A_ * exp(k_slope_ * ctl.compPostureTask->eval().norm()) + C_; // Exponential
    mc_rtc::log::info("[MonodzukuriKinovaDemo] Stiffness posture adjusted to: {}", stiffness_posture_);
    damping_posture_ = 3*std::sqrt(stiffness_posture_);
    ctl.compPostureTask->stiffness(stiffness_posture_);
    ctl.compPostureTask->damping(damping_posture_);
  }

  if(ctl.compEETask->eval().norm() < 1.0)
  {
    stiffness_task_ = (A_ * exp(k_slope_ * ctl.compEETask->eval().norm()) + C_)/2; // Exponential
    mc_rtc::log::info("[MonodzukuriKinovaDemo] Stiffness task adjusted to: {}", stiffness_task_);
    damping_task_ = 4*std::sqrt(stiffness_task_);
    ctl.compEETask->positionTask->stiffness(stiffness_task_);
    ctl.compEETask->orientationTask->stiffness(stiffness_task_);
    ctl.compEETask->positionTask->damping(damping_task_);
    ctl.compEETask->orientationTask->damping(damping_task_);
  }

  // if(ctl.compEETask->eval().norm() < 0.1 && ctl.compPostureTask->eval().norm() < 0.25)
  if(ctl.compPostureTask->eval().norm() < 0.2)
  {
    mc_rtc::log::info("[MonodzukuriKinovaDemo] Reached waypoint {} of {}", wayPointIndex_, ctl.wayPoints.size());
    if(!taskEEInSolver_) {
      mc_rtc::log::info("[MonodzukuriKinovaDemo] Adding end-effector task to solver");
      ctl.solver().addTask(ctl.compEETask);
      taskEEInSolver_ = true;
    }
    timeOut_ = false;
    counter_ = 0.0;

    if(wayPointIndex_ < ctl.wayPoints.size())
    {
      stiffness_posture_ = stiffnessMin_;
      damping_posture_ = 3*std::sqrt(stiffness_posture_);
      stiffness_task_ = stiffnessMin_/2; // Reduce stiffness for end-effector task
      damping_task_ = 4*std::sqrt(stiffness_task_);

      ctl.compEETask->set_ef_pose(ctl.wayPoints[wayPointIndex_].second);
      ctl.compEETask->positionTask->stiffness(stiffness_task_);
      ctl.compEETask->orientationTask->stiffness(stiffness_task_);
      ctl.compEETask->positionTask->damping(damping_task_);
      ctl.compEETask->orientationTask->damping(damping_task_);

      ctl.compPostureTask->target(ctl.wayPoints[wayPointIndex_].first);
      ctl.compPostureTask->stiffness(stiffness_posture_);
      ctl.compPostureTask->damping(damping_posture_);
      wayPointIndex_++;
    }
    else
    {
      ctl.kinestheticTeachingHasBeenPlayed_ = true;
      mc_rtc::log::info("[MonodzukuriKinovaDemo] All waypoints reached.");
      output("OK");
      return true;
    }
  }

  // if the target was reached, move to the next waypoint
  // if(ctl.compEETask->eval().norm() < 0.1 && ctl.compPostureTask->eval().norm() < 0.1)

  return false;
}

void MonodzukuriKinovaDemo_wayPointDCompliant::teardown(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  ctl.compEETask->reset();
  ctl.compEETask->positionTask->stiffness(400);
  ctl.compEETask->positionTask->weight(10000);
  ctl.compEETask->orientationTask->stiffness(400);
  ctl.compEETask->orientationTask->weight(10000);
  ctl.compPostureTask->stiffness(0.0);
  ctl.compPostureTask->damping(2.0);
  ctl.compPostureTask->weight(1);
  // if(ctl.kinestheticTeachingHasBeenPlayed_) ctl.wayPoints.clear();
  ctl.gui()->removeElements(this);
}

void MonodzukuriKinovaDemo_wayPointDCompliant::addGui(
    mc_control::fsm::Controller &ctl_) {
  auto &ctl = static_cast<MonodzukuriKinovaDemo &>(ctl_);
  auto gui = ctl.gui();
  gui->addElement(
      this, {"Kinesthetic Teaching"},
      mc_rtc::gui::Label("Current Waypoint Index", [this]() { return wayPointIndex_; }),
      mc_rtc::gui::Label("Current Stiffness", [this]() { return stiffness_posture_; }),
      mc_rtc::gui::NumberInput(
          "Stiffness Min", [this]() { return stiffnessMin_; },
          [this](double s) { 
            stiffnessMin_ = s;
            A_ = (-stiffnessMax_ + stiffnessMin_) / (exp(k_slope_) - 1);
            C_ = stiffnessMax_ - A_; 
          }),
      mc_rtc::gui::NumberInput(
          "Stiffness Max", [this]() { return stiffnessMax_; },
          [this](double s) { 
            stiffnessMax_ = s; 
            A_ = (-stiffnessMax_ + stiffnessMin_) / (exp(k_slope_) - 1);
            C_ = stiffnessMax_ - A_;
          }),
      mc_rtc::gui::NumberInput(
          "Slope for stiffness adjustment", [this]() { return k_slope_; },
          [this](double slope) { 
            k_slope_ = slope; 
            A_ = (-stiffnessMax_ + stiffnessMin_) / (exp(k_slope_) - 1);
            C_ = stiffnessMax_ - A_;
          })
        );
}

EXPORT_SINGLE_STATE("MonodzukuriKinovaDemo_wayPointDCompliant",
                    MonodzukuriKinovaDemo_wayPointDCompliant)
