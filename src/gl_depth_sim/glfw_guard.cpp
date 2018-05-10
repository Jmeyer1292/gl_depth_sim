#include <gl_depth_sim/glfw_guard.h>
#include <GLFW/glfw3.h>

gl_depth_sim::GlfwGuard::GlfwGuard()
{
  glfwInit();
}

gl_depth_sim::GlfwGuard::~GlfwGuard()
{
  glfwTerminate();
}
