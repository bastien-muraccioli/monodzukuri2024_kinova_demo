#include "MonodzukuriKinovaDemo.h"

MonodzukuriKinovaDemo::MonodzukuriKinovaDemo(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // Initialize the velocity damper parameters (closed-loop by default)
  dt_ = dt;
  xsiOff_ = 0.0;
  m_ = 2.0;
  lambda_ = 100.0;
  velocityDamperFlag_ = true;
  closeLoopVelocityDamper_ = true;

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  mc_rtc::log::success("MonodzukuriKinovaDemo init done ");
}

bool MonodzukuriKinovaDemo::run()
{
  return mc_control::fsm::Controller::run();
}

void MonodzukuriKinovaDemo::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}

void MonodzukuriKinovaDemo::updateConstraints(bool closeLoop)
{
  if(closeLoop)
  {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
    solver().addConstraintSet(dynamicsConstraint);
    selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});

    mc_rtc::log::info("[RALExpController] Close Loop Velocity damper is enabled");
  }
  else
  {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, dt_, {0.1, 0.01, 0.5}, 0.9, false, true));
    solver().addConstraintSet(dynamicsConstraint);
    selfCollisionConstraint->setCollisionsDampers(solver(), {0.0, 0.0});

    mc_rtc::log::info("[RALExpController] Close Loop Velocity damper is deactivated");
  }
}

void MonodzukuriKinovaDemo::updateConstraints(void)
{
    if(m_ < 1.0 || lambda_ < 1.0)
    {
      solver().removeConstraintSet(dynamicsConstraint);
      dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, dt_, {0.1, 0.01, 0.5}, 0.9, false, true));
      solver().addConstraintSet(dynamicsConstraint);
      selfCollisionConstraint->setCollisionsDampers(solver(), {0.0, 0.0});
      velocityDamperFlag_ = false;
      closeLoopVelocityDamper_ = false;
    }
    else // Close loop velocity damper
    {
      solver().removeConstraintSet(dynamicsConstraint);
      dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
      solver().addConstraintSet(dynamicsConstraint);
      selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});
      velocityDamperFlag_ = true;
      closeLoopVelocityDamper_ = true;
    }
    mc_rtc::log::info("[RALExpController] Constraints updated");
}