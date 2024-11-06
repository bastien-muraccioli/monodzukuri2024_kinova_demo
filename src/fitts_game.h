#pragma once

// clang-format off
#include <mc_rtc/log/Logger.h>
#include <GL/glew.h>
#include <GL/gl.h>
// clang-format on

#include <GL/glut.h>
#include <GLFW/glfw3.h>
#include <string>
#include <utility>

typedef std::pair<float, float> position;

struct FittsData
{
  int idx;
  float distance;
  float radius;
  float reach_time;
};

class FittsGame
{
public:
  FittsGame();
  void run();

  // Getters and setters
  std::pair<int, int> getWinSize() const;
  void setWinSize(const std::pair<int, int> & size);
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

  void addToLogger(mc_rtc::Logger & logger);

private:
  // Window related
  std::pair<int, int> win_size;
  GLFWwindow * window;
  GLuint circleVAO;
  GLuint circleVBO;

  //
  position pointer_pos;
  float pointer_radius;
  position robot_pos;
  float robot_radius;
  position target_pos;
  float target_radius;
  int target_idx;
  float target_radius_max = 30.0;
  float target_radius_min = 200;
  double proj;

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
  double target_distance = 400;
  Eigen::Vector2d initial_direction;

  position newRandomPose(float radius);
  void initCircle(float radius, int num_segments);
  void drawCircle(position pos, float r, int num_segments);
  void deleteCircle(void);
  void renderText(float x, float y, const std::string & text, void * font);
  void renderFPS(float x, float y, double fps);
  void renderTargetIdx(float x, float y, int idx);
};
