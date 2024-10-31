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

  // Task variables
  Eigen::MatrixXd taskOrientation_; // Rotation Matrix
  Eigen::Vector3d taskPosition_;

  // Joypad controller
  bool joypadTriggerControlFlag = false;  // When you press the trigger button R2 (RT in the pluggin), you enter in torque control mode otherwise itś position control mode
  bool joypadComplianceModeFlag = false;  // When you press the right pad button, you enter in compliance mode
  bool joypadNullSpaceModeFlag = false;   // When you press the up pad button, you enter in nullspace mode
  bool joypadCompliSinusModeFlag = false; // When you press the bottom pad button, you enter in compliance sinus mode
  bool joypadMinJerkModeFlag = false;     // When you press the left pad button, you enter in minimum jerk mode

  bool changeModeAvailable = true;        
  bool changeModeRequest = false;

private:
  mc_rtc::Configuration config_;
  std::vector<mc_rbdyn::Collision> collisions_;
  void getPostureTarget(void);

  void joypadManager(void);
  
  // State index
  int stateIndex_;

  // Dynamics velocity damper private parameters
  double dt_;
  bool velocityDamperFlag_;
  double xsiOff_;

  bool upPadLastState_ = false;
  bool downPadLastState_ = false;
  bool rightPadLastState_ = false;
  bool leftPadLastState_ = false;
  bool r1ButtonLastState_ = false; // Change torque mode in Custom (RB button in the pluggin)
  bool l1ButtonLastState_ = false; // Change torque mode in Default (LB button in the pluggin)
};
