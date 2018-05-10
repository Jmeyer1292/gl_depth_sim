#ifndef GL_DEPTH_SIM_GLFW_GUARD_H
#define GL_DEPTH_SIM_GLFW_GUARD_H

namespace gl_depth_sim
{

/**
 * @brief Calls glfwInit() in constructor and glfwTerminate() in destructor. Used to ensure that
 * these are the first and last methods called so we don't destroy openGL objects after GLFW has
 * been terminated.
 */
struct GlfwGuard
{
  GlfwGuard();

  ~GlfwGuard();
};

}

#endif // GLFW_GUARD_H
