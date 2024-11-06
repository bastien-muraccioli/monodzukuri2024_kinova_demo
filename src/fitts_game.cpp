#include "./fitts_game.h"
#include <cmath>
#include <iomanip>
#include <sstream>

// Constructor
FittsGame::FittsGame()
: win_size(900, 900), pointer_radius(3.0f), wait_time(1.0f), pointer_pos(450.0f, 450.0f), target_idx(-1)
{
}

// Run the game loop
void FittsGame::run()
{
  // Initialize GLUT
  int dummy_argc = 1;
  char * dummy_argv[1] = {(char *)"dummy"};
  glutInit(&dummy_argc, dummy_argv);

  // Initialize GLFW
  if(!glfwInit())
  {
    exit(-1);
  }

  // Create a windowed mode window and its OpenGL context
  window = glfwCreateWindow(win_size.first, win_size.second, "Fitts's game", NULL, NULL);
  if(!window)
  {
    glfwTerminate();
    exit(-1);
  }

  // Make the window's context current
  glfwMakeContextCurrent(window);

  // Initialize GLEW
  glewExperimental = GL_TRUE; // Enable experimental features
  GLenum err = glewInit();
  if(GLEW_OK != err)
  {
    exit(-1);
  }

  // Disable vsync
  glfwSwapInterval(0);

  // Random seed for target position
  std::srand(static_cast<unsigned>(time(0)));

  initCircle(1.0f, 100);

  // Initial target
  target_radius = target_radius_min + (std::rand() / (float)RAND_MAX) * (target_radius_max - target_radius_min);
  target_pos = newRandomPose(target_radius);
  new_target = true;
  double dx = target_pos.first - pointer_pos.first;
  double dy = target_pos.second - pointer_pos.second;
  float initial_distance = std::sqrt(dx * dx + dy * dy);
  initial_direction(0) = dx;
  initial_direction(1) = dy;

  // Main loop
  while(!glfwWindowShouldClose(window))
  {
    glClear(GL_COLOR_BUFFER_BIT);
    glClearColor(1.0, 1.0, 1.0, 1.0);

    // Set up the projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0.0, win_size.first, 0.0, win_size.second, -1.0, 1.0);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    // Draw target position circle
    glColor3f(0.0, 0.8, 0.3); // Green
    drawCircle(target_pos, target_radius, 100);

    // Draw current position circle
    glColor3f(0.0, 0.0, 0.0); // Red
    drawCircle(pointer_pos, pointer_radius, 100);

    // Check if current position is within the target circle
    dx = target_pos.first - pointer_pos.first;
    dy = target_pos.second - pointer_pos.second;
    float radius_diff = target_radius - pointer_radius;
    double current_time = glfwGetTime();
    proj = (dx * initial_direction(0) + dy * initial_direction(1)) / initial_direction.norm();
    if(dx * dx + dy * dy <= radius_diff * radius_diff)
    {
      if(current_time - last_time >= wait_time)
      {
        // Calculate time to reach the target
        reach_time = current_time - start_time - wait_time;

        last_target_data.idx = target_idx;
        last_target_data.distance = target_distance;
        last_target_data.reach_time = reach_time;
        last_target_data.radius = target_radius;

        // Generate new target
        if(target_idx < 10)
        {
          target_radius = 40;
        }
        else
        {
          target_radius = target_radius_min + (std::rand() / (float)RAND_MAX) * (target_radius_max - target_radius_min);
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
    }
    else
    {
      last_time = glfwGetTime(); // Reset timer if outside target
    }

    // Calculate and render FPS
    static double previous_time = glfwGetTime(); // Initialize with the current time
    static int frame_count = 0;
    double elapsed_time = current_time - previous_time;
    frame_count++;

    if(elapsed_time >= 0.05)
    { // Update FPS every second
      fps = frame_count / elapsed_time;
      frame_count = 0; // Reset frame count
      previous_time = current_time; // Update previous_time to the current time
    }
    renderFPS(10, win_size.second - 30, fps); // Draw FPS in the top-left corner

    // Render target index
    renderTargetIdx(win_size.first - 200, win_size.second - 30,
                    target_idx); // Draw target index in the top-right corner

    glfwSwapBuffers(window);
    glfwPollEvents();
  }

  // Clean up
  deleteCircle();
  glfwDestroyWindow(window);
  glfwTerminate();
}

// Getters and setters
std::pair<int, int> FittsGame::getWinSize() const
{
  return win_size;
}

void FittsGame::setWinSize(const std::pair<int, int> & size)
{
  win_size = size;
}

float FittsGame::getRobotRadius() const
{
  return robot_radius;
}

void FittsGame::setRobotRadius(float radius)
{
  robot_radius = radius;
}

void FittsGame::setRobotPosition(float x, float y, float offset_x, float offset_y)
{
  pointer_pos.first = win_size.first * (x - offset_x + robot_radius) / (2.0 * robot_radius);
  pointer_pos.second = win_size.second * (y - offset_y + robot_radius) / (2.0 * robot_radius);
}

float FittsGame::getPointerRadius() const
{
  return pointer_radius;
}

void FittsGame::setPointerRadius(float radius)
{
  pointer_radius = radius;
}

void FittsGame::setPointerPosition(float x, float y)
{
  pointer_pos.first = x;
  pointer_pos.second = y;
}

float FittsGame::getWaitTime() const
{
  return wait_time;
}

void FittsGame::setWaitTime(float time)
{
  wait_time = time;
}

bool FittsGame::getNewTargetBool(void)
{
  return new_target;
}

void FittsGame::clearNewTargetBool(void)
{
  new_target = false;
}

position FittsGame::getTargetPos(void)
{
  return target_pos;
}

float FittsGame::getTargetRadius(void)
{
  return target_radius;
}

int FittsGame::getTargetIdx(void)
{
  return target_idx;
}

FittsData FittsGame::getLastData(void)
{
  return last_target_data;
}

void FittsGame::addToLogger(mc_rtc::Logger & logger)
{
  logger.addLogEntry("FittsGame_position", [this]() { return Eigen::Vector2d(pointer_pos.first, pointer_pos.second); });
  logger.addLogEntry("FittsGame_reach_time", [this]() { return reach_time; });
  logger.addLogEntry("FittsGame_target_distance", [this]() { return target_distance; });
  logger.addLogEntry("FittsGame_target_radius", [this]() { return target_radius; });
  logger.addLogEntry("FittsGame_target_initial_direction", [this]() { return initial_direction; });
  logger.addLogEntry("FittsGame_target_idx", [this]() { return target_idx; });
  logger.addLogEntry("FittsGame_projection", [this]() { return proj; });
}

position FittsGame::newRandomPose(float radius)
{
  float dx, dy, radius_sum, new_x, new_y, angle;
  bool valid;
  do
  {
    angle = -M_PI + (std::rand() / (float)RAND_MAX) * (2.0 * M_PI);
    new_x = pointer_pos.first + target_distance * cosf(angle);
    new_y = pointer_pos.second + target_distance * sinf(angle);
    valid = new_x > radius and new_x < win_size.first - radius and new_y > radius and new_y < win_size.second - radius;
  } while(not valid);
  return position(new_x, new_y);
}

void FittsGame::initCircle(float radius, int num_segments)
{
  std::vector<float> vertices;
  for(int i = 0; i <= num_segments; ++i)
  {
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
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);

  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void *)0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);
  glBindVertexArray(0);
}

void FittsGame::drawCircle(position pos, float r, int num_segments)
{
  glBindVertexArray(circleVAO);
  glPushMatrix();
  glTranslatef(pos.first, pos.second, 0.0f);
  glScalef(r, r, 1.0f);
  glDrawArrays(GL_TRIANGLE_FAN, 0, num_segments + 1);
  glPopMatrix();
  glBindVertexArray(0);
}

void FittsGame::deleteCircle(void)
{
  glDeleteVertexArrays(1, &circleVAO);
  glDeleteBuffers(1, &circleVBO);
}

void FittsGame::renderText(float x, float y, const std::string & text, void * font)
{
  glRasterPos2f(x, y);
  for(char c : text)
  {
    glutBitmapCharacter(font, c);
  }
}

void FittsGame::renderFPS(float x, float y, double fps)
{
  std::stringstream ss;
  ss << "FPS: " << std::fixed << std::setprecision(2) << fps;
  renderText(x, y, ss.str(), GLUT_BITMAP_HELVETICA_18);
}

void FittsGame::renderTargetIdx(float x, float y, int idx)
{
  std::stringstream ss;
  ss << "Target Index: " << idx;
  renderText(x, y, ss.str(), GLUT_BITMAP_HELVETICA_18);
}
