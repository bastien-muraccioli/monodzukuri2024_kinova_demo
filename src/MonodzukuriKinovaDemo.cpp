#include "MonodzukuriKinovaDemo.h"

MonodzukuriKinovaDemo::MonodzukuriKinovaDemo(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config, Backend::TVM)
{
  // Initialize the velocity damper parameters (closed-loop by default)
  dt_ctrl = dt;
  xsiOff_ = 0.0;
  m_ = 1.8;
  lambda_ = 70.0;
  velocityDamperFlag_ = true;
  closeLoopVelocityDamper_ = true;

  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(robots(), 0, {0.1, 0.01, xsiOff_, m_, lambda_}, 0.9, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize the future tasks values
  compEETask = std::make_shared<mc_tasks::CompliantEndEffectorTask>("FT_sensor_mounting", robots(),
                                                                    robot().robotIndex(), 1.0, 10000.0);
  postureHome = {{"joint_1", {0}}, {"joint_2", {0.262}}, {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                   {"joint_5", {0}}, {"joint_6", {0.96}},  {"joint_7", {1.57}}};
  postureTarget = {{"joint_1", {0}}, {"joint_2", {0.59}}, {"joint_3", {3.14}}, {"joint_4", {-1.56}},
                   {"joint_5", {0}}, {"joint_6", {0.483}},  {"joint_7", {1.57}}};
  postureCalibration = {{"joint_1", {0}}, {"joint_2", {0.32}}, {"joint_3", {3.14}}, {"joint_4", {-1.46}},
                   {"joint_5", {0}}, {"joint_6", {-1.33}},  {"joint_7", {1.57}}};

  // Initalize the current task
  taskOrientation_ = Eigen::Quaterniond(1,-1,-1,-1).normalized().toRotationMatrix();
  taskPosition_ = Eigen::Vector3d(0.68, 0.0, 0.45);
  posture_target_log.setZero(robot().mb().nrJoints());
  solver().removeTask(getPostureTask(robot().name()));
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->stiffness(0.0);
  compPostureTask->damping(4.0);
  compPostureTask->target(postureHome);
  solver().addTask(compPostureTask);

  //OpenGL GUI
  // game.run();
  gameThread = std::thread(std::bind([this]() { game.run(); }));
  game.setRobotRadius(robot_radius);
  game.addToLogger(logger());
  while(not game.getNewTargetBool())
    ;

  // Datastore
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("TorqueMode", "Custom");
  game.setJRLTorque(true);
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr { return compPostureTask; });

  // GUI
  gui()->addElement({"Controller"}, mc_rtc::gui::Checkbox("Close Loop Velocity Damper", 
                    [this]() { return velocityDamperFlag_; }, [this]() { velocityDamperFlag_ = !velocityDamperFlag_; }));
  gui()->addElement({"Controller"}, mc_rtc::gui::NumberInput("m", [this]() { return m_; },
                    [this](double m) { m_ = m; }));
  gui()->addElement({"Controller"}, mc_rtc::gui::NumberInput("lambda", [this]() { return lambda_; },
                    [this](double lambda) { lambda_ = lambda; }));
  gui()->addElement({"Controller"}, mc_rtc::gui::Button("SEND", [this]() { updateConstraints(); }));

  gui()->addElement({"Controller"}, mc_rtc::gui::Checkbox("Trigger", 
                  [this]() { return joypadTriggerControlFlag; }, [this]() { joypadTriggerControlFlag = !joypadTriggerControlFlag; }));
  gui()->addElement({"Controller"}, mc_rtc::gui::Checkbox("Activate", 
                  [this]() { return activateFlag; }, [this]() { activateFlag = !activateFlag; }));
  

  mc_rtc::log::success("MonodzukuriKinovaDemo init done ");
}

bool MonodzukuriKinovaDemo::run()
{
  
  // Update the velocity damper constraints
  if (velocityDamperFlag_ && !closeLoopVelocityDamper_)
  {
    updateConstraints(true);
    closeLoopVelocityDamper_ = true;
  }
  else if (!velocityDamperFlag_ && closeLoopVelocityDamper_)
  {
    updateConstraints(false);
    closeLoopVelocityDamper_ = false;
  }

  // Joypad manager
  if(datastore().get<bool>("Joystick::connected"))
  {
    joypadManager();
  }

  // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if(ctrl_mode.compare("Position") == 0)
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  }
  else
  {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }

  return false;
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
      new mc_solver::DynamicsConstraint(robots(), 0, dt_ctrl, {0.1, 0.01, 0.5}, 0.9, false, true));
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
        new mc_solver::DynamicsConstraint(robots(), 0, dt_ctrl, {0.1, 0.01, 0.5}, 0.9, false, true));
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

void MonodzukuriKinovaDemo::getPostureTarget(void)
{
  posture_target_log = rbd::dofToVector(robot().mb(), compPostureTask->posture());
}

void MonodzukuriKinovaDemo::joypadManager(void)
{
  auto & buttonFunc = datastore().get<std::function<bool(joystickButtonInputs button)>>("Joystick::Button");
  auto & triggerFunc = datastore().get<std::function<double(joystickAnalogicInputs)>>("Joystick::Trigger");
  bool upPadState = datastore().get<bool>("Joystick::UpPad");
  bool downPadState = datastore().get<bool>("Joystick::DownPad");
  bool leftPadState = datastore().get<bool>("Joystick::LeftPad");
  bool rightPadState = datastore().get<bool>("Joystick::RightPad");

  if (buttonFunc(A) && buttonFunc(A) != xButtonLastState_) // X Button
  {
    activateFlag = !activateFlag;
  }

  if (buttonFunc(Y) && buttonFunc(Y) != triangleButtonLastState_) // Triangle Button
  {
    compliantFlag = !compliantFlag;
  }

  if (buttonFunc(RB) && buttonFunc(RB) != r1ButtonLastState_) // R1 Button
  {
    datastore().assign<std::string>("TorqueMode", "Custom");
    mc_rtc::log::info("Torque mode: Custom");
    game.setJRLTorque(true);
  }
  else if (buttonFunc(LB) && buttonFunc(LB) != l1ButtonLastState_) // L1 Button
  {
    datastore().assign<std::string>("TorqueMode", "Default");
    mc_rtc::log::info("Torque mode: Default");
    game.setJRLTorque(false);
  }

  if (triggerFunc(RT) < 1.0) // R2 Trigger
  {
    joypadTriggerControlFlag = true;
  }
  else
  {
    joypadTriggerControlFlag = false;
  }

  if (changeModeAvailable && !changeModeRequest)
  {
    joypadNullSpaceModeFlag = false;
    joypadCompliSinusModeFlag = false;
    joypadComplianceModeFlag = false;
    joypadMinJerkModeFlag = false;

    if (upPadState && upPadState != upPadLastState_)
    {
      joypadNullSpaceModeFlag = true;
      changeModeRequest = true;
    }
    else if(downPadState && downPadState != downPadLastState_)
    {
      joypadCompliSinusModeFlag = true;
      changeModeRequest = true;
    }
    else if(rightPadState && rightPadState != rightPadLastState_)
    {
      joypadComplianceModeFlag = true;
      changeModeRequest = true;
    }
    else if(leftPadState && leftPadState != leftPadLastState_)
    {
      joypadMinJerkModeFlag = true;
      changeModeRequest = true;
    }
  }
  r1ButtonLastState_ = buttonFunc(RB);
  l1ButtonLastState_ = buttonFunc(LB);
  upPadLastState_ = upPadState;
  downPadLastState_ = downPadState;
  rightPadLastState_ = rightPadState;
  leftPadLastState_ = leftPadState;
  xButtonLastState_ = buttonFunc(A);
  triangleButtonLastState_= buttonFunc(Y);
  // mc_rtc::log::info("TorqueMode {}; NullSpaceMode {}; CompliSinusMode {}; ComplianceMode {}; MinJerkMode {}", 
  //                   joypadTorqueModeFlag, joypadNullSpaceModeFlag, joypadCompliSinusModeFlag, joypadComplianceModeFlag, joypadMinJerkModeFlag);
}