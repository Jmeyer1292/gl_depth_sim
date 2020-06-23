#ifndef GL_DEPTH_SIM_SIM_DEPTH_CAMERA_H
#define GL_DEPTH_SIM_SIM_DEPTH_CAMERA_H

#include "gl_depth_sim/camera_properties.h"
#include "gl_depth_sim/glfw_guard.h"
#include "gl_depth_sim/renderable_mesh.h"
#include "gl_depth_sim/shader_program.h"

#include <memory>
#include <vector>
#include <map>

// Foward declare GLFWWindow
struct GLFWwindow;

namespace gl_depth_sim
{

/**
 * @brief Utility data structure used internally by @class SimDepthCamera
 */
struct RenderableObjectState
{
  std::unique_ptr<RenderableMesh> mesh;
  Eigen::Isometry3d pose;
};

/**
 * @brief This is the primary means by which you should use the depth camera simulator. This class creates a GLFW
 * window using the properties specified to the constructor. See @class CameraProperties. Users should then add objects
 * to the scene by calling "add". Finally, to generate a depth image, call "render" with the cameras pose. The
 * resulting object can be converted to more ROS-y formats using the utilities under the gl_depth_sim/interfaces/
 * headers.
 *
 * @warning This class is NOT THREAD SAFE! Furthermore, you must call render() from the SAME THREAD THAT CALLED THE
 * CONSTRUCTOR! This is because OpenGL creates per-thread context information.
 */
class SimDepthCamera
{
public:
  /**
   * @brief Creates an OpenGL context and window using the given camera parameters. Only one of these should be created
   * at a time.
   */
  SimDepthCamera(const CameraProperties& camera);
  ~SimDepthCamera();

  /**
   * @brief Invokes OpenGL to render the current scene given a camera pose. Will walk all active models and create a
   * depth image from their composite.
   * @param pose The position of the camera in ROS standard coordinates (+Z down camera LoS, +Y down the image). Note
   * that this is different from OpenGL's frame of Y up, -Z down camera.
   * @return A data structure that contains a linearized depth data matrix. See @class DepthImage.
   */
  DepthImage render(const Eigen::Isometry3d& pose);

  /**
   * @brief Adds a triangle mesh to the scene with the given pose in world coordinates.
   */
  bool add(const std::string mesh_id, const Mesh& mesh, const Eigen::Isometry3d& pose);
  bool add(const Mesh& mesh, const Eigen::Isometry3d& pose);

  /**
   * @brief Moves a triangle mesh within the scene given an identifier string and a pose in world coordinates.
   */
  bool move(const std::string mesh_id, const Eigen::Isometry3d& pose);

private:
  void initGLFW();
  void createGLFramebuffer();

  // Controls the starting and stopping of glfw - This MUST come before any objects containing
  // opengl handles in this list.
  GlfwGuard guard_;

  // State information
  CameraProperties camera_;
  Eigen::Matrix4d proj_;
  std::map<std::string, RenderableObjectState> objects_;

  // OpenGL context info
  GLFWwindow* window_;
  std::unique_ptr<ShaderProgram> depth_program_;
  unsigned int fbo_;
};

}

#endif
