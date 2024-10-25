#pragma once

#include <mc_control/fsm/Controller.h>

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

private:
  mc_rtc::Configuration config_;

  // Dynamics velocity damper private parameters
  double dt_;
  bool velocityDamperFlag_;
  double xsiOff_;
  const double i_ = 0.03;
  const double s_ = 0.015;
  const double d_ = 0.0;

};
