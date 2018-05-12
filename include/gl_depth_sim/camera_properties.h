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

/**
 * @brief A buffer for raw depth data. Depth buffers are fetched from opengl into this buffer and
 * are then linearized.
 */
struct DepthImage
{
  int rows, cols;
  std::vector<float> data;

  DepthImage() = default;
  DepthImage(int rows, int cols) : rows{rows}, cols{cols}, data(rows * cols) {}

  /**
   * @brief Returns the distance at a given row and column in the image given ROS coordinates! This means
   * y down the height of the screen. OpenGL stores data Y UP so here we have to flip the row input.
   */
  float distance(int row, int col) const
  {
    const int y = rows - row - 1;
    const int x = col;
    return data[y * cols + x];
  }
};

}

#endif // GL_DEPTH_SIM_CAMERA_PROPERTIES_H
