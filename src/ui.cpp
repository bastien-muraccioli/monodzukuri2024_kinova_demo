#include "ui.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <cmath>
#include <iomanip>
#include <sstream>

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

// Constructor
FittsGame::FittsGame()
    : win_size(1920, 1080), win_fitts_size(570, 570), pointer_radius(3.0f),
      wait_time(1.0f), target_idx(-1) {}

// Run the game loop
void FittsGame::run() {
  // Initialize variables
  min_fitts_x = FITTS_WINDOW_OFFSET_X;
  min_fitts_y = FITTS_WINDOW_OFFSET_Y;
  max_fitts_x = win_fitts_size.first + FITTS_WINDOW_OFFSET_X;
  max_fitts_y = win_fitts_size.first + FITTS_WINDOW_OFFSET_Y;
  pointer_pos.first = (min_fitts_x + max_fitts_x) / 2;
  pointer_pos.second = (min_fitts_y + max_fitts_y) / 2;

  mc_rtc::log::info("pointer_pos: ({}, {})", pointer_pos.first,
                    pointer_pos.second);

  // // Initialize GLUT
  // int dummy_argc = 1;
  // char * dummy_argv[1] = {(char *)"dummy"};
  // glutInit(&dummy_argc, dummy_argv);

  // Initialize GLFW
  if (!glfwInit()) {
    mc_rtc::log::error("Failed to initialize GLFW");
    exit(-1);
  }

  // Create a windowed mode window and its OpenGL context
  window = glfwCreateWindow(win_size.first, win_size.second,
                            "Futur Monodzukuri 2024", nullptr, nullptr);
  if (!window) {
    mc_rtc::log::error("Failed to create GLFW window");
    glfwTerminate();
    exit(-1);
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLEW (required for OpenGL functions)
  glewExperimental = GL_TRUE; // Enable experimental features
  if (glewInit() != GLEW_OK) {
    mc_rtc::log::error("Failed to initialize GLEW");
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(-1);
  }

  loadingTextures();

  // // Disable vsync
  glfwSwapInterval(0);

  // // Set up the projection
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  // glOrtho(0.0, win_size.first, win_size.second, 0.0, -1.0, 1.0);
  glOrtho(0.0, win_size.first, 0.0, win_size.second, -1.0, 1.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  // Enable blending for transparency
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

  // // Random seed for target position
  std::srand(static_cast<unsigned>(time(0)));

  initCircle(1.0f, 100);

  // // Initial target
  target_radius =
      target_radius_min +
      (std::rand() / (float)RAND_MAX) * (target_radius_max - target_radius_min);
  target_pos = newRandomPose(target_radius);
  new_target = true;
  double dx = target_pos.first - pointer_pos.first;
  double dy = target_pos.second - pointer_pos.second;
  float initial_distance = std::sqrt(dx * dx + dy * dy);
  initial_direction(0) = dx;
  initial_direction(1) = dy;

  // Swap front and back buffers
  glfwSwapBuffers(window);

  // Poll for and process events
  glfwPollEvents();

  // Main loop
  while (!glfwWindowShouldClose(window)) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glLoadIdentity();

    manageBackground();

    if (control_mode == MJ_MODE) {

      // Draw target position circle
      glColor3f(0.0, 0.8, 0.3); // Green
      drawCircle(target_pos, target_radius, 100);

      // Draw current position circle
      glColor3f(0.0, 0.0, 0.0); // Red
      drawCircle(pointer_pos, pointer_radius, 100);

      // Check if current position is within the target circle
      dx = target_pos.first - pointer_pos.first;
      dy = target_pos.second - pointer_pos.second;
      float radius_diff = 1.25 * target_radius - pointer_radius;
      double current_time = glfwGetTime();
      proj = (dx * initial_direction(0) + dy * initial_direction(1)) /
             initial_direction.norm();
      if (dx * dx + dy * dy <= radius_diff * radius_diff) {
        if (current_time - last_time >= wait_time) {
          // Calculate time to reach the target
          reach_time = current_time - start_time - wait_time;

          last_target_data.idx = target_idx;
          last_target_data.distance = target_distance;
          last_target_data.reach_time = reach_time;
          last_target_data.radius = target_radius;

          // Generate new target
          if (target_idx < 10) {
            target_radius = 40;
          } else {
            target_radius =
                target_radius_min + (std::rand() / (float)RAND_MAX) *
                                        (target_radius_max - target_radius_min);
          }
          target_pos = newRandomPose(target_radius);
          target_idx++;

          // Reset timer and start time
          last_time = current_time;
          start_time = current_time;

          // Update initial distance to the new target
          dx = target_pos.first - pointer_pos.first;
          dy = target_pos.second - pointer_pos.second;
          initial_direction(0) = dx;
          initial_direction(1) = dy;
          new_target = true;
        }
      } else {
        last_time = glfwGetTime(); // Reset timer if outside target
      }

      // Calculate and render FPS
      static double previous_time =
          glfwGetTime(); // Initialize with the current time
      static int frame_count = 0;
      double elapsed_time = current_time - previous_time;
      frame_count++;

      if (elapsed_time >= 0.05) { // Update FPS every second
        fps = frame_count / elapsed_time;
        frame_count = 0; // Reset frame count
        previous_time =
            current_time; // Update previous_time to the current time
      }

      // renderFPS(100, 300, fps); // Draw FPS in the top-left corner

      // // Render target index
      // renderTargetIdx(100, 300,
      //                 target_idx); // Draw target index in the top-right
      //                 corner
    }

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Clean up
  deleteTextures();
  deleteCircle();
  glfwDestroyWindow(window);
  glfwTerminate();
}

void FittsGame::manageBackground(void) {

  if(uiInEnglish_)
  {
    switch (control_mode) {
    case MC_MODE:
      if (jrl_torque) {
        renderTexture(mc_jrlID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(mc_defaultID_en, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case DC_MODE:
      if (jrl_torque) {
        renderTexture(dc_jrlID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(dc_defaultID_en, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case FC_MODE:
      if (jrl_torque) {
        renderTexture(fc_jrlID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(fc_defaultID_en, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case MJ_MODE:
      if (jrl_torque) {
        renderTexture(mj_jrlID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(mj_defaultID_en, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case NS_MODE:
      if (jrl_torque) {
        renderTexture(ns_jrlID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(ns_defaultID_en, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case MTC_MODE:
      if (jrl_torque) {
        renderTexture(mtc_jrlID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(mtc_defaultID_en, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case INTRO:
      renderTexture(introID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      break;
    default:
      renderTexture(introID_en, win_size.first / 2, win_size.second / 2, 1080.0f);
      break;
    }
  }
  else{
        switch (control_mode) {
    case MC_MODE:
      if (jrl_torque) {
        renderTexture(mc_jrlID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(mc_defaultID_jp, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case DC_MODE:
      if (jrl_torque) {
        renderTexture(dc_jrlID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(dc_defaultID_jp, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case FC_MODE:
      if (jrl_torque) {
        renderTexture(fc_jrlID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(fc_defaultID_jp, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case MJ_MODE:
      if (jrl_torque) {
        renderTexture(mj_jrlID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(mj_defaultID_jp, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case NS_MODE:
      if (jrl_torque) {
        renderTexture(ns_jrlID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(ns_defaultID_jp, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case MTC_MODE:
      if (jrl_torque) {
        renderTexture(mtc_jrlID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      } else {
        renderTexture(mtc_defaultID_jp, win_size.first / 2, win_size.second / 2,
                      1080.0f);
      }
      break;
    case INTRO:
      renderTexture(introID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      break;
    default:
      renderTexture(introID_jp, win_size.first / 2, win_size.second / 2, 1080.0f);
      break;
    }
  }
  
}

void FittsGame::loadingTextures(void) {
  // mc_rtc::log::info("Loading texture from {}", );
  introID_en = loadTexture(getPath("intro.png", true).c_str());
  mc_jrlID_en = loadTexture(getPath("mc-jrl.png", true).c_str());
  dc_jrlID_en = loadTexture(getPath("dc-jrl.png", true).c_str());
  fc_jrlID_en = loadTexture(getPath("fc-jrl.png", true).c_str());
  mj_jrlID_en = loadTexture(getPath("mj-jrl.png", true).c_str());
  ns_jrlID_en = loadTexture(getPath("ns-jrl.png", true).c_str());
  mtc_jrlID_en = loadTexture(getPath("mtc-jrl.png", true).c_str());
  mc_defaultID_en = loadTexture(getPath("mc-default.png", true).c_str());
  dc_defaultID_en = loadTexture(getPath("dc-default.png", true).c_str());
  fc_defaultID_en = loadTexture(getPath("fc-default.png", true).c_str());
  mj_defaultID_en = loadTexture(getPath("mj-default.png", true).c_str());
  ns_defaultID_en = loadTexture(getPath("ns-default.png", true).c_str());
  mtc_defaultID_en = loadTexture(getPath("mtc-default.png", true).c_str());

  // Japanese textures
  introID_jp = loadTexture(getPath("intro.png", false).c_str());
  mc_jrlID_jp = loadTexture(getPath("mc-jrl.png", false).c_str());
  dc_jrlID_jp = loadTexture(getPath("dc-jrl.png", false).c_str());
  fc_jrlID_jp = loadTexture(getPath("fc-jrl.png", false).c_str());
  mj_jrlID_jp = loadTexture(getPath("mj-jrl.png", false).c_str());
  ns_jrlID_jp = loadTexture(getPath("ns-jrl.png", false).c_str());
  mtc_jrlID_jp = loadTexture(getPath("mtc-jrl.png", false).c_str());
  mc_defaultID_jp = loadTexture(getPath("mc-default.png", false).c_str());
  dc_defaultID_jp = loadTexture(getPath("dc-default.png", false).c_str());
  fc_defaultID_jp = loadTexture(getPath("fc-default.png", false).c_str());
  mj_defaultID_jp = loadTexture(getPath("mj-default.png", false).c_str());
  ns_defaultID_jp = loadTexture(getPath("ns-default.png", false).c_str());
  mtc_defaultID_jp = loadTexture(getPath("mtc-default.png", false).c_str());

}

void FittsGame::deleteTextures(void) {
  glDeleteTextures(1, &introID_en);
  glDeleteTextures(1, &mc_jrlID_en);
  glDeleteTextures(1, &dc_jrlID_en);
  glDeleteTextures(1, &fc_jrlID_en);
  glDeleteTextures(1, &mj_jrlID_en);
  glDeleteTextures(1, &ns_jrlID_en);
  glDeleteTextures(1, &mtc_jrlID_en);
  glDeleteTextures(1, &mc_defaultID_en);
  glDeleteTextures(1, &dc_defaultID_en);
  glDeleteTextures(1, &fc_defaultID_en);
  glDeleteTextures(1, &mj_defaultID_en);
  glDeleteTextures(1, &ns_defaultID_en);
  glDeleteTextures(1, &mtc_defaultID_en);

  glDeleteTextures(1, &introID_jp);
  glDeleteTextures(1, &mc_jrlID_jp);
  glDeleteTextures(1, &dc_jrlID_jp);
  glDeleteTextures(1, &fc_jrlID_jp);
  glDeleteTextures(1, &mj_jrlID_jp);
  glDeleteTextures(1, &ns_jrlID_jp);
  glDeleteTextures(1, &mtc_jrlID_jp);
  glDeleteTextures(1, &mc_defaultID_jp);
  glDeleteTextures(1, &dc_defaultID_jp);
  glDeleteTextures(1, &fc_defaultID_jp);
  glDeleteTextures(1, &mj_defaultID_jp);
  glDeleteTextures(1, &ns_defaultID_jp);
  glDeleteTextures(1, &mtc_defaultID_jp);

}

void FittsGame::setControlMode(int mode) { control_mode = mode; }

void FittsGame::setJRLTorque(bool torque) { jrl_torque = torque; }

// Getters and setters
std::pair<int, int> FittsGame::getWinSize() const { return win_size; }

void FittsGame::setWinSize(const std::pair<int, int> &size) { win_size = size; }

float FittsGame::getRobotRadius() const { return robot_radius; }

void FittsGame::setRobotRadius(float radius) { robot_radius = radius; }

void FittsGame::setRobotPosition(float x, float y, float offset_x,
                                 float offset_y) {
  pointer_pos.first =
      (win_size.first * (x - offset_x + robot_radius) / (2.0 * robot_radius));
  pointer_pos.second =
      (win_size.second * (y - offset_y + robot_radius) / (2.0 * robot_radius));
}

float FittsGame::getPointerRadius() const { return pointer_radius; }

void FittsGame::setPointerRadius(float radius) { pointer_radius = radius; }

void FittsGame::setPointerPosition(float x, float y) {
  pointer_pos.first = x;
  pointer_pos.second = y;
}

float FittsGame::getWaitTime() const { return wait_time; }

void FittsGame::setWaitTime(float time) { wait_time = time; }

bool FittsGame::getNewTargetBool(void) { return new_target; }

void FittsGame::clearNewTargetBool(void) { new_target = false; }

position FittsGame::getTargetPos(void) { return target_pos; }

float FittsGame::getTargetRadius(void) { return target_radius; }

int FittsGame::getTargetIdx(void) { return target_idx; }

FittsData FittsGame::getLastData(void) { return last_target_data; }

void FittsGame::addToLogger(mc_rtc::Logger &logger) {
  logger.addLogEntry("FittsGame_position", [this]() {
    return Eigen::Vector2d(pointer_pos.first, pointer_pos.second);
  });
  logger.addLogEntry("FittsGame_reach_time", [this]() { return reach_time; });
  logger.addLogEntry("FittsGame_target_distance",
                     [this]() { return target_distance; });
  logger.addLogEntry("FittsGame_target_radius",
                     [this]() { return target_radius; });
  logger.addLogEntry("FittsGame_target_initial_direction",
                     [this]() { return initial_direction; });
  logger.addLogEntry("FittsGame_target_idx", [this]() { return target_idx; });
  logger.addLogEntry("FittsGame_projection", [this]() { return proj; });
}

position FittsGame::newRandomPose(float radius) {
  float dx, dy, radius_sum, new_x, new_y, angle;
  bool valid;
  do {
    angle = -M_PI + (std::rand() / (float)RAND_MAX) * (2.0 * M_PI);
    new_x = pointer_pos.first + target_distance * cosf(angle);
    new_y = pointer_pos.second + target_distance * sinf(angle);
    valid = min_fitts_x + radius < new_x and new_x < max_fitts_x - radius and
            min_fitts_y + radius < new_y and new_y < max_fitts_y - radius;
    mc_rtc::log::info("new_x: {}, new_y: {}, valid: {}", new_x, new_y, valid);
  } while (not valid);
  return position(new_x, new_y);
}

void FittsGame::initCircle(float radius, int num_segments) {
  std::vector<float> vertices;
  for (int i = 0; i <= num_segments; ++i) {
    float theta = 2.0f * 3.1415926f * float(i) / float(num_segments);
    float x = radius * cosf(theta);
    float y = radius * sinf(theta);
    vertices.push_back(x);
    vertices.push_back(y);
  }

  glGenVertexArrays(1, &circleVAO);
  glGenBuffers(1, &circleVBO);

  glBindVertexArray(circleVAO);
  glBindBuffer(GL_ARRAY_BUFFER, circleVBO);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float),
               vertices.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void FittsGame::drawCircle(position pos, float r, int num_segments) {
  glBindVertexArray(circleVAO);
  glPushMatrix();
  glTranslatef(pos.first, pos.second, 0.0f);
  glScalef(1.25 * r, 1.25 * r, 1.0f);
  glDrawArrays(GL_TRIANGLE_FAN, 0, num_segments + 1);
  glPopMatrix();
  glBindVertexArray(0);
}

void FittsGame::deleteCircle(void) {
  glDeleteVertexArrays(1, &circleVAO);
  glDeleteBuffers(1, &circleVBO);
}

void FittsGame::renderText(float x, float y, const std::string &text,
                           void *font) {
  glRasterPos2f(x, y);
  for (char c : text) {
    glutBitmapCharacter(font, c);
  }
}

void FittsGame::renderFPS(float x, float y, double fps) {
  std::stringstream ss;
  ss << "FPS: " << std::fixed << std::setprecision(2) << fps;
  renderText(x, y, ss.str(), GLUT_BITMAP_HELVETICA_18);
}

void FittsGame::renderTargetIdx(float x, float y, int idx) {
  std::stringstream ss;
  ss << "Target Index: " << idx;
  renderText(x, y, ss.str(), GLUT_BITMAP_HELVETICA_18);
}

std::string FittsGame::getPath(const std::string &filename, bool english) {
  // Get the directory of the current source file
  std::filesystem::path source_file = __FILE__;
  std::filesystem::path source_dir =
      source_file.parent_path(); // Get the source directory

  std::filesystem::path font_path;

  // Construct the font path relative to the source directory
  if (english) {
    font_path = source_dir / "assets-en" / filename;
  }
  else {
    font_path = source_dir / "assets-jp" / filename;
  }

  return font_path.string();
}

GLuint FittsGame::loadTexture(const char *filename) {
  int width, height, channels;
  unsigned char *image = stbi_load(filename, &width, &height, &channels, 0);
  if (!image) {
    mc_rtc::log::error("Failed to load image");
    return 0;
  }

  GLuint textureID;

  glGenTextures(1, &textureID);
  glBindTexture(GL_TEXTURE_2D, textureID);

  // Set texture parameters
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);

  // Load the texture to OpenGL
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0,
               (channels == 4 ? GL_RGBA : GL_RGB), GL_UNSIGNED_BYTE, image);
  glGenerateMipmap(GL_TEXTURE_2D);

  stbi_image_free(image);
  return textureID;
}

void FittsGame::renderTexture(GLuint texture, float x, float y, float scale) {
  glColor4f(1.0, 1.0, 1.0, 1.0);
  // Get the texture's width and height (assuming you have these values stored
  // when loading the texture)
  int width, height;

  glBindTexture(GL_TEXTURE_2D, texture);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_WIDTH, &width);
  glGetTexLevelParameteriv(GL_TEXTURE_2D, 0, GL_TEXTURE_HEIGHT, &height);

  // Calculate the texture's aspect ratio
  float aspectRatio = (float)width / (float)height;
  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, texture);

  // Position and scale the quad
  glPushMatrix();
  glTranslatef(x, y, 0.0f);
  glScalef(scale * aspectRatio, scale, 1.0f);

  // Draw the textured quad
  glBegin(GL_QUADS);
  glTexCoord2f(0.0f, 0.0f);
  glVertex2f(-0.5f, 0.5f);
  glTexCoord2f(1.0f, 0.0f);
  glVertex2f(0.5f, 0.5f);
  glTexCoord2f(1.0f, 1.0f);
  glVertex2f(0.5f, -0.5f);
  glTexCoord2f(0.0f, 1.0f);
  glVertex2f(-0.5f, -0.5f);

  // // Define the texture coordinate (0, 1) and the top-left corner of the
  // quad. glTexCoord2f(0.0f, 1.0f); glVertex2f(-0.5f, 0.5f);

  // // Define the texture coordinate (1, 1) and the top-right corner of the
  // quad. glTexCoord2f(1.0f, 1.0f); glVertex2f(0.5f, 0.5f);

  // // Define the texture coordinate (1, 0) and the bottom-right corner of the
  // quad. glTexCoord2f(1.0f, 0.0f); glVertex2f(0.5f, -0.5f);

  // // Define the texture coordinate (0, 0) and the bottom-left corner of the
  // quad. glTexCoord2f(0.0f, 0.0f); glVertex2f(-0.5f, -0.5f);
  glEnd();

  glPopMatrix();
  glBindTexture(GL_TEXTURE_2D, 0);
  glDisable(GL_TEXTURE_2D);
}

void FittsGame::changeLanguage(void) 
{ 
  uiInEnglish_ = !uiInEnglish_;
}
