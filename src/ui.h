#pragma once

#include <filesystem>

// clang-format off
#include <mc_rtc/log/Logger.h>
#include <GL/glew.h>
#include <GL/gl.h>
#include <png.h>

// clang-format on

#include <GL/glut.h>
#include <GLFW/glfw3.h>

#include <fstream>
#include <iostream>
#include <string>
#include <utility>

#define FITTS_WINDOW_OFFSET_X 940.0
#define FITTS_WINDOW_OFFSET_Y 90.0
#define MC_MODE 0
#define DC_MODE 1
#define FC_MODE 2
#define MJ_MODE 3
#define NS_MODE 4
#define INTRO 5
#define MTC_MODE 6

typedef std::pair<float, float> position;

struct FittsData {
  int idx;
  float distance;
  float radius;
  float reach_time;
};

class FittsGame {
public:
  FittsGame();
  void run();

  // Getters and setters
  std::pair<int, int> getWinSize() const;
  void setWinSize(const std::pair<int, int> &size);
  float getRobotRadius() const;
  void setRobotRadius(float radius);
  void setRobotPosition(float x, float y, float offset_x, float offset_y);
  float getPointerRadius() const;
  void setPointerRadius(float radius);
  void setPointerPosition(float x, float y);
  float getWaitTime() const;
  void setWaitTime(float time);
  bool getNewTargetBool(void);
  void clearNewTargetBool(void);
  position getTargetPos(void);
  float getTargetRadius(void);
  int getTargetIdx(void);
  FittsData getLastData(void);

  void setControlMode(int mode);
  void setJRLTorque(bool jrl_torque);

  void addToLogger(mc_rtc::Logger &logger);
  void changeLanguage(void);

private:
  // Window related
  std::pair<int, int> win_size;
  std::pair<int, int> win_fitts_size;
  GLFWwindow *window;
  GLuint circleVAO;
  GLuint circleVBO;

  // English textures

  GLuint introID_en; // Background intro

  GLuint mc_jrlID_en; // Background Moving Compliance, JRL torque
  GLuint dc_jrlID_en; // Background Dual Compliance, JRL torque
  GLuint fc_jrlID_en; // Background Full Compliance, JRL torque
  GLuint mj_jrlID_en; // Background MinJerk, JRL torque
  GLuint ns_jrlID_en; // Background Null Space, JRL torque
  GLuint mtc_jrlID_en; // Background Moving Null Space, JRL torque

  GLuint mc_defaultID_en; // Background Moving Compliance, Default torque
  GLuint dc_defaultID_en; // Background Dual Compliance, Default torque
  GLuint fc_defaultID_en; // Background Full Compliance, Default torque
  GLuint mj_defaultID_en; // Background MinJerk, Default torque
  GLuint ns_defaultID_en; // Background Null Space, Default torque
  GLuint mtc_defaultID_en; // Background Moving Null Space, Default torque

  // Japanese textures

  GLuint introID_jp; // Background intro

  GLuint mc_jrlID_jp; // Background Moving Compliance, JRL torque
  GLuint dc_jrlID_jp; // Background Dual Compliance, JRL torque
  GLuint fc_jrlID_jp; // Background Full Compliance, JRL torque
  GLuint mj_jrlID_jp; // Background MinJerk, JRL torque
  GLuint ns_jrlID_jp; // Background Null Space, JRL torque
  GLuint mtc_jrlID_jp; // Background Moving Null Space, JRL torque

  GLuint mc_defaultID_jp; // Background Moving Compliance, Default torque
  GLuint dc_defaultID_jp; // Background Dual Compliance, Default torque
  GLuint fc_defaultID_jp; // Background Full Compliance, Default torque
  GLuint mj_defaultID_jp; // Background MinJerk, Default torque
  GLuint ns_defaultID_jp; // Background Null Space, Default torque
  GLuint mtc_defaultID_jp; // Background Moving Null Space, Default torque

  void manageBackground(void);

  void deleteTextures(void);
  void loadingTextures(void);

  GLuint loadTexture(const char *filename);
  void renderTexture(GLuint texture, float x, float y, float scale);

  std::string getPath(const std::string &filename, bool english);
  bool uiInEnglish_ = true;

  int control_mode = INTRO;
  bool jrl_torque = true;

  //
  position pointer_pos;
  float pointer_radius;
  position robot_pos;
  float robot_radius;
  position target_pos;
  float target_radius;
  int target_idx;
  float target_radius_max = 30;
  float target_radius_min = 80.0;
  double proj;

  int min_fitts_x;
  int max_fitts_x;
  int min_fitts_y;
  int max_fitts_y;

  //
  double fps;
  bool new_target;
  FittsData last_target_data;

  //
  double start_time;
  double current_time;
  double last_time;
  double reach_time;
  double wait_time;
  double target_distance = 250;
  Eigen::Vector2d initial_direction;

  position newRandomPose(float radius);
  void initCircle(float radius, int num_segments);
  void drawCircle(position pos, float r, int num_segments);
  void deleteCircle(void);
  void renderText(float x, float y, const std::string &text, void *font);
  void renderFPS(float x, float y, double fps);
  void renderTargetIdx(float x, float y, int idx);
};
