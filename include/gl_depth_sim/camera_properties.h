#ifndef GL_DEPTH_SIM_CAMERA_PROPERTIES_H
#define GL_DEPTH_SIM_CAMERA_PROPERTIES_H

#include <vector>

namespace gl_depth_sim
{

struct CameraProperties
{
  int width, height;

  float fx, fy;
  float cx, cy;

  float z_near, z_far;
};

struct DepthImage
{
  int rows, cols;
  std::vector<float> data;
};

}

#endif // GL_DEPTH_SIM_CAMERA_PROPERTIES_H
