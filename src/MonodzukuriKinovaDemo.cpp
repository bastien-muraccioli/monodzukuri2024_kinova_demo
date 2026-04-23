#include "MonodzukuriKinovaDemo.h"

MonodzukuriKinovaDemo::MonodzukuriKinovaDemo(
    mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration &config)
    : mc_control::fsm::Controller(rm, dt, config, Backend::TVM) {

  // Initialize the velocity damper parameters (closed-loop by default)
  dt_ctrl = dt;
  xsiOff_ = 0.0;
  m_ = 1.8;
  lambda_ = 70.0;
  velocityDamperFlag_ = true;
  closeLoopVelocityDamper_ = true;
  jointNumber = robot(robots()[0].name()).refJointOrder().size();

  tool_frame = config("tool_frame", (std::string) "FT_sensor_wrench");
  shoulder_frame = config("shoulder_frame", (std::string) "forearm_link");

  mc_rtc::log::info("[MonodzukuriKinovaDemo] Tool frame: {}", tool_frame);
  // Initialize the constraints
  selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});
  dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
      new mc_solver::DynamicsConstraint(robots(), 0, timeStep,
                                        {0.1, 0.01, xsiOff_}, {m_, lambda_},
                                        0.9, false, true));
  solver().addConstraintSet(dynamicsConstraint);

  // Initialize the future tasks values
  compEETask = std::make_shared<mc_tasks::CompliantEndEffectorTask>(
      tool_frame, robots(), robot().robotIndex(), 1.0, 10000.0);

  postureHome = {{"joint_1", {0}},    {"joint_2", {0.262}},
                 {"joint_3", {3.14}}, {"joint_4", {-2.269}},
                 {"joint_5", {0}},    {"joint_6", {0.96}},
                 {"joint_7", {1.57}}};
  postureTarget = {{"joint_1", {0}},    {"joint_2", {0.58}},
                   {"joint_3", {3.14}}, {"joint_4", {-1.84}},
                   {"joint_5", {0}},    {"joint_6", {0.85}},
                   {"joint_7", {1.57}}};
  postureCalibration = {{"joint_1", {0}},    {"joint_2", {0.32}},
                        {"joint_3", {3.14}}, {"joint_4", {-1.46}},
                        {"joint_5", {0}},    {"joint_6", {-1.33}},
                        {"joint_7", {1.57}}};

  // Initalize the current task
  taskOrientation_ =
      Eigen::Quaterniond(1, -1, -1, -1).normalized().toRotationMatrix();
  taskPosition_ = Eigen::Vector3d(0.6, 0.0, 0.4);
  posture_target_log.setZero(robot().mb().nrJoints());
  solver().removeTask(getPostureTask(robot().name()));
  compPostureTask = std::make_shared<mc_tasks::CompliantPostureTask>(
      solver(), robot().robotIndex(), 1, 1);
  compPostureTask->reset();
  compPostureTask->stiffness(0.0);
  compPostureTask->damping(4.0);
  compPostureTask->target(postureHome);
  solver().addTask(compPostureTask);

  // OpenGL GUI
  //  game.run();
  gameThread = std::thread(std::bind([this]() { game.run(); }));
  game.setRobotRadius(robot_radius);
  game.addToLogger(logger());
  while (not game.getNewTargetBool())
    ;

  // Datastore
  datastore().make<std::string>("ControlMode", "Position");
  datastore().make<std::string>("TorqueMode", "Custom");
  game.setJRLTorque(true);
  datastore().make_call("getPostureTask", [this]() -> mc_tasks::PostureTaskPtr {
    return compPostureTask;
  });
  gui()->addElement({"UI"}, mc_rtc::gui::Checkbox(
                                "English (Checked) or Japanese (Unchecked)",
                                [this]() { return uiInEnglish_; },
                                [this]() { uiInEnglish_ = !uiInEnglish_; }));

  // GUI
  gui()->addElement(
      {"Controller"},
      mc_rtc::gui::Checkbox(
          "Close Loop Velocity Damper",
          [this]() { return velocityDamperFlag_; },
          [this]() { velocityDamperFlag_ = !velocityDamperFlag_; }));
  gui()->addElement({"Controller"}, mc_rtc::gui::NumberInput(
                                        "m", [this]() { return m_; },
                                        [this](double m) { m_ = m; }));
  gui()->addElement({"Controller"},
                    mc_rtc::gui::NumberInput(
                        "lambda", [this]() { return lambda_; },
                        [this](double lambda) { lambda_ = lambda; }));
  gui()->addElement({"Controller"}, mc_rtc::gui::Button("SEND", [this]() {
                      updateConstraints();
                    }));

  gui()->addElement(
      {"Controller"},
      mc_rtc::gui::Checkbox(
          "Trigger", [this]() { return joypadTriggerControlFlag; },
          [this]() { joypadTriggerControlFlag = !joypadTriggerControlFlag; }));
  gui()->addElement({"Controller"},
                    mc_rtc::gui::Checkbox(
                        "Activate", [this]() { return crossButtonFlag; },
                        [this]() { crossButtonFlag = !crossButtonFlag; }));
  gui()->addElement(
      {"Controller", "Joystick"},
      mc_rtc::gui::Button("Press A",
                          [this]() { simulateAButtonPress_ = true; }),
      mc_rtc::gui::Button("Press B",
                          [this]() { simulateBButtonPress_ = true; }),
      mc_rtc::gui::Button("Press X",
                          [this]() { simulateXButtonPress_ = true; }),
      mc_rtc::gui::Button("Press Y",
                          [this]() { simulateYButtonPress_ = true; }),
      mc_rtc::gui::Button("Press LB",
                          [this]() { simulateLBButtonPress_ = true; }),
      mc_rtc::gui::Button("Press RB",
                          [this]() { simulateRBButtonPress_ = true; }),
      mc_rtc::gui::Button("Press START",
                          [this]() { simulateStartButtonPress_ = true; }),
      mc_rtc::gui::Button("Press SELECT",
                          [this]() { simulateSelectButtonPress_ = true; }),
      mc_rtc::gui::Button("Pad Up", [this]() { simulateUpPadPress_ = true; }),
      mc_rtc::gui::Button("Pad Down",
                          [this]() { simulateDownPadPress_ = true; }),
      mc_rtc::gui::Button("Pad Left",
                          [this]() { simulateLeftPadPress_ = true; }),
      mc_rtc::gui::Button("Pad Right",
                          [this]() { simulateRightPadPress_ = true; }),
      mc_rtc::gui::Checkbox(
          "Hold RT", [this]() { return simulateRTPressed_; },
          [this]() { simulateRTPressed_ = !simulateRTPressed_; }));

  mc_rtc::log::success("MonodzukuriKinovaDemo init done ");
}

bool MonodzukuriKinovaDemo::run() {

  if (languageFlag_ != uiInEnglish_) {
    game.changeLanguage();
    languageFlag_ = uiInEnglish_;
  }

  // Update the velocity damper constraints
  if (velocityDamperFlag_ && !closeLoopVelocityDamper_) {
    updateConstraints(true);
    closeLoopVelocityDamper_ = true;
  } else if (!velocityDamperFlag_ && closeLoopVelocityDamper_) {
    updateConstraints(false);
    closeLoopVelocityDamper_ = false;
  }

  // Joypad manager
  if (datastore().get<bool>("Joystick::connected") ||
      hasSimulatedJoystickInput()) {
    joypadManager();
  }

  if (joypadReturnToInitialFlag) {
    if (running() && executor_.state() != "MonodzukuriKinovaDemo_Initial") {
      mc_rtc::log::info("[MonodzukuriKinovaDemo] Returning to Initial");
      interrupt();
    } else if (!running()) {
      if (resume("MonodzukuriKinovaDemo_Initial")) {
        mc_rtc::log::info(
            "[MonodzukuriKinovaDemo] Resumed Initial after interruption");
      }
      joypadReturnToInitialFlag = false;
      changeModeAvailable = true;
      changeModeRequest = false;
      joypadNullSpaceModeFlag = false;
      joypadCompliSinusModeFlag = false;
      joypadComplianceModeFlag = false;
      joypadMinJerkModeFlag = false;
      joypadBoxDemoModeFlag = false;
    } else {
      joypadReturnToInitialFlag = false;
    }
  }

  // Update the solver depending on the control mode
  auto ctrl_mode = datastore().get<std::string>("ControlMode");
  if (ctrl_mode.compare("Position") == 0) {
    return mc_control::fsm::Controller::run(mc_solver::FeedbackType::OpenLoop);
  } else {
    return mc_control::fsm::Controller::run(
        mc_solver::FeedbackType::ClosedLoopIntegrateReal);
  }

  return false;
}

void MonodzukuriKinovaDemo::reset(
    const mc_control::ControllerResetData &reset_data) {
  mc_control::fsm::Controller::reset(reset_data);
}

void MonodzukuriKinovaDemo::updateConstraints(bool closeLoop) {
  if (closeLoop) {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, timeStep,
                                          {0.1, 0.01, xsiOff_}, {m_, lambda_},
                                          0.9, false, true));
    solver().addConstraintSet(dynamicsConstraint);
    selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});

    mc_rtc::log::info(
        "[RALExpController] Close Loop Velocity damper is enabled");
  } else {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, dt_ctrl,
                                          {0.1, 0.01, 0.5}, 0.9, false, true));
    solver().addConstraintSet(dynamicsConstraint);
    selfCollisionConstraint->setCollisionsDampers(solver(), {0.0, 0.0});

    mc_rtc::log::info(
        "[RALExpController] Close Loop Velocity damper is deactivated");
  }
}

void MonodzukuriKinovaDemo::updateConstraints(void) {
  if (m_ < 1.0 || lambda_ < 1.0) {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, dt_ctrl,
                                          {0.1, 0.01, 0.5}, 0.9, false, true));
    solver().addConstraintSet(dynamicsConstraint);
    selfCollisionConstraint->setCollisionsDampers(solver(), {0.0, 0.0});
    velocityDamperFlag_ = false;
    closeLoopVelocityDamper_ = false;
  } else // Close loop velocity damper
  {
    solver().removeConstraintSet(dynamicsConstraint);
    dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
        new mc_solver::DynamicsConstraint(robots(), 0, timeStep,
                                          {0.1, 0.01, xsiOff_}, {m_, lambda_},
                                          0.9, false, true));
    solver().addConstraintSet(dynamicsConstraint);
    selfCollisionConstraint->setCollisionsDampers(solver(), {m_, lambda_});
    velocityDamperFlag_ = true;
    closeLoopVelocityDamper_ = true;
  }
  mc_rtc::log::info("[RALExpController] Constraints updated");
}

void MonodzukuriKinovaDemo::getPostureTarget(void) {
  posture_target_log =
      rbd::dofToVector(robot().mb(), compPostureTask->posture());
}

bool MonodzukuriKinovaDemo::consumeSimulatedPress(bool &flag) {
  const bool pressed = flag;
  flag = false;
  return pressed;
}

bool MonodzukuriKinovaDemo::hasSimulatedJoystickInput(void) const {
  return simulateAButtonPress_ || simulateBButtonPress_ ||
         simulateXButtonPress_ || simulateYButtonPress_ ||
         simulateLBButtonPress_ || simulateRBButtonPress_ ||
         simulateStartButtonPress_ || simulateSelectButtonPress_ ||
         simulateUpPadPress_ || simulateDownPadPress_ ||
         simulateLeftPadPress_ || simulateRightPadPress_ || simulateRTPressed_;
}

void MonodzukuriKinovaDemo::resetModeSwitchState(void) {
  joypadNullSpaceModeFlag = false;
  joypadCompliSinusModeFlag = false;
  joypadComplianceModeFlag = false;
  joypadMinJerkModeFlag = false;
  joypadBoxDemoModeFlag = false;
  joypadReturnToInitialFlag = false;
  changeModeAvailable = true;
  changeModeRequest = false;

  auto reset_last_states = [this]() {
    upPadLastState_ = false;
    downPadLastState_ = false;
    leftPadLastState_ = false;
    rightPadLastState_ = false;
    r1ButtonLastState_ = false;
    l1ButtonLastState_ = false;
    xButtonLastState_ = false;
    squareButtonLastState_ = false;
    triangleButtonLastState_ = false;
    circleButtonLastState_ = false;
    startButtonLastState_ = false;
    selectButtonLastState_ = false;
  };

  if (!datastore().has("Joystick::connected") ||
      !datastore().get<bool>("Joystick::connected") ||
      !datastore().has("Joystick::Button")) {
    reset_last_states();
    return;
  }

  auto &buttonFunc =
      datastore().get<std::function<bool(joystickButtonInputs button)>>(
          "Joystick::Button");
  upPadLastState_ = datastore().get<bool>("Joystick::UpPad");
  downPadLastState_ = datastore().get<bool>("Joystick::DownPad");
  leftPadLastState_ = datastore().get<bool>("Joystick::LeftPad");
  rightPadLastState_ = datastore().get<bool>("Joystick::RightPad");
  r1ButtonLastState_ = buttonFunc(RB);
  l1ButtonLastState_ = buttonFunc(LB);
  xButtonLastState_ = buttonFunc(A);
  squareButtonLastState_ = buttonFunc(Y);
  triangleButtonLastState_ = buttonFunc(X);
  circleButtonLastState_ = buttonFunc(B);
  startButtonLastState_ = buttonFunc(START);
  selectButtonLastState_ = buttonFunc(SELECT);
}

void MonodzukuriKinovaDemo::joypadManager(void) {
  auto &buttonFunc =
      datastore().get<std::function<bool(joystickButtonInputs button)>>(
          "Joystick::Button");
  auto &triggerFunc =
      datastore().get<std::function<double(joystickAnalogicInputs)>>(
          "Joystick::Trigger");
  const bool aButtonState =
      buttonFunc(A) || consumeSimulatedPress(simulateAButtonPress_);
  const bool bButtonState =
      buttonFunc(B) || consumeSimulatedPress(simulateBButtonPress_);
  const bool xButtonState =
      buttonFunc(X) || consumeSimulatedPress(simulateXButtonPress_);
  const bool yButtonState =
      buttonFunc(Y) || consumeSimulatedPress(simulateYButtonPress_);
  const bool lbButtonState =
      buttonFunc(LB) || consumeSimulatedPress(simulateLBButtonPress_);
  const bool rbButtonState =
      buttonFunc(RB) || consumeSimulatedPress(simulateRBButtonPress_);
  const bool startButtonState =
      buttonFunc(START) || consumeSimulatedPress(simulateStartButtonPress_);
  const bool selectButtonState =
      buttonFunc(SELECT) || consumeSimulatedPress(simulateSelectButtonPress_);
  const bool upPadState = datastore().get<bool>("Joystick::UpPad") ||
                          consumeSimulatedPress(simulateUpPadPress_);
  const bool downPadState = datastore().get<bool>("Joystick::DownPad") ||
                            consumeSimulatedPress(simulateDownPadPress_);
  const bool leftPadState = datastore().get<bool>("Joystick::LeftPad") ||
                            consumeSimulatedPress(simulateLeftPadPress_);
  const bool rightPadState = datastore().get<bool>("Joystick::RightPad") ||
                             consumeSimulatedPress(simulateRightPadPress_);

  if (aButtonState && aButtonState != xButtonLastState_) // X Button
  {
    crossButtonFlag = !crossButtonFlag;
  }

  if (yButtonState && yButtonState != squareButtonLastState_) // Square Button
  {
    squareButtonFlag = !squareButtonFlag;
  }

  if (xButtonState &&
      xButtonState != triangleButtonLastState_) // Triangle Button
  {
    triangleButtonFlag = !triangleButtonFlag;
  }

  if (bButtonState && bButtonState != circleButtonLastState_) // Circle Button
  {
    circleButtonFlag = !circleButtonFlag;
  }

  if (rbButtonState && rbButtonState != r1ButtonLastState_) // R1 Button
  {
    datastore().assign<std::string>("TorqueMode", "Custom");
    mc_rtc::log::info("Torque mode: Custom");
    game.setJRLTorque(true);
  } else if (lbButtonState && lbButtonState != l1ButtonLastState_) // L1 Button
  {
    datastore().assign<std::string>("TorqueMode", "Default");
    mc_rtc::log::info("Torque mode: Default");
    game.setJRLTorque(false);
  }

  if (triggerFunc(RT) < 1.0 || simulateRTPressed_) // R2 Trigger
  {
    joypadTriggerControlFlag = true;
  } else {
    joypadTriggerControlFlag = false;
  }

  if (changeModeAvailable && !changeModeRequest) {
    joypadNullSpaceModeFlag = false;
    joypadCompliSinusModeFlag = false;
    joypadComplianceModeFlag = false;
    joypadMinJerkModeFlag = false;
    joypadBoxDemoModeFlag = false;

    if (upPadState && upPadState != upPadLastState_) {
      joypadNullSpaceModeFlag = true;
      changeModeRequest = true;
    } else if (downPadState && downPadState != downPadLastState_) {
      joypadCompliSinusModeFlag = true;
      changeModeRequest = true;
    } else if (rightPadState && rightPadState != rightPadLastState_) {
      joypadComplianceModeFlag = true;
      changeModeRequest = true;
    } else if (leftPadState && leftPadState != leftPadLastState_) {
      joypadMinJerkModeFlag = true;
      changeModeRequest = true;
    } else if (startButtonState && startButtonState != startButtonLastState_) {
      joypadBoxDemoModeFlag = true;
      changeModeRequest = true;
    }
  }

  if (selectButtonState && selectButtonState != selectButtonLastState_) {
    joypadReturnToInitialFlag = true;
  }
  r1ButtonLastState_ = rbButtonState;
  l1ButtonLastState_ = lbButtonState;
  upPadLastState_ = upPadState;
  downPadLastState_ = downPadState;
  rightPadLastState_ = rightPadState;
  leftPadLastState_ = leftPadState;
  xButtonLastState_ = aButtonState;
  squareButtonLastState_ = yButtonState;
  triangleButtonLastState_ = xButtonState;
  circleButtonLastState_ = bButtonState;
  startButtonLastState_ = startButtonState;
  selectButtonLastState_ = selectButtonState;
  // mc_rtc::log::info("TorqueMode {}; NullSpaceMode {}; CompliSinusMode {};
  // ComplianceMode {}; MinJerkMode {}",
  //                   joypadTorqueModeFlag, joypadNullSpaceModeFlag,
  //                   joypadCompliSinusModeFlag, joypadComplianceModeFlag,
  //                   joypadMinJerkModeFlag);
}
