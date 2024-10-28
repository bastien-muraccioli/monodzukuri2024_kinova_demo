#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_rbdyn/VirtualTorqueSensor.h>
#include <mc_tasks/CompliantEndEffectorTask.h>
#include <mc_tasks/CompliantPostureTask.h>
#include <mc_rbdyn/Collision.h>

#include "api.h"

#define HIGH_RESIDUAL_GAIN 10.0
#define LOW_RESIDUAL_GAIN 0.5

struct MonodzukuriKinovaDemo_DLLAPI MonodzukuriKinovaDemo : public mc_control::fsm::Controller
{
  MonodzukuriKinovaDemo(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config);

  bool run() override;

  void reset(const mc_control::ControllerResetData & reset_data) override;

  // Update the Dynamics and the Collisions constraints for the controller
  void updateConstraints(bool closeLoop); 
  void updateConstraints(void);

  // Dynamics velocity damper parameters
  double m_;
  double lambda_;
  bool closeLoopVelocityDamper_;

  // Tasks
  std::shared_ptr<mc_tasks::CompliantPostureTask> compPostureTask;
  std::shared_ptr<mc_tasks::CompliantEndEffectorTask> compEETask;

  // Targets
  std::map<std::string, std::vector<double>> postureHome;
  std::map<std::string, std::vector<double>> postureTarget;
  Eigen::VectorXd posture_target_log;

  // State variables
  bool moveNextState;
  int currentSequence;
  std::string sequenceOutput;
  bool waitingForInput;

  // Task variables
  Eigen::MatrixXd taskOrientation_; // Rotation Matrix
  Eigen::Vector3d taskPosition_;

private:
  mc_rtc::Configuration config_;
  std::vector<mc_rbdyn::Collision> collisions_;
  void getPostureTarget(void);
  
  // State index
  int stateIndex_;

  // Dynamics velocity damper private parameters
  double dt_;
  bool velocityDamperFlag_;
  double xsiOff_;
  const double i_ = 0.03;
  const double s_ = 0.015;
  const double d_ = 0.0;

};
