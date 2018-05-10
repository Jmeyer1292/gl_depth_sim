#ifndef GL_DEPTH_SIM_SIM_DEPTH_CAMERA_H
#define GL_DEPTH_SIM_SIM_DEPTH_CAMERA_H

#include "gl_depth_sim/camera_properties.h"
#include "gl_depth_sim/renderable_mesh.h"
#include "gl_depth_sim/shader_program.h"
#include <GLFW/glfw3.h>

#include <memory>
#include <vector>

namespace gl_depth_sim
{

struct RenderableObjectState
{
  std::unique_ptr<RenderableMesh> mesh;
  Eigen::Affine3d pose;
};

class SimDepthCamera
{
public:
  SimDepthCamera(const CameraProperties& camera);
  ~SimDepthCamera();

  DepthImage render(const Eigen::Affine3d& pose);

  bool add(const Mesh& mesh, const Eigen::Affine3d& pose);

private:
  // State information
  CameraProperties camera_;
  std::vector<RenderableObjectState> objects_;
  Eigen::Matrix4d proj_;

  // OpenGL context info
  GLFWwindow* window_;
  std::unique_ptr<ShaderProgram> depth_program_;
  unsigned int fbo_;
};

}

#endif
