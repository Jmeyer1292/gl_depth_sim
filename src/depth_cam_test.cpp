#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"

#include <iostream>

int main()
{
  std::string line;

  gl_depth_sim::CameraProperties props;
  props.width = 640;
  props.height = 480;
  props.fx = 550.0f;
  props.fy = 550.0f;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25f;
  props.z_far = 10.0f;

  gl_depth_sim::SimDepthCamera sim (props);

  auto mesh_ptr = gl_depth_sim::loadMesh("/home/jmeyer/untitled.stl");

  sim.add(*mesh_ptr, Eigen::Affine3d::Identity());

  while (std::getline(std::cin, line))
  {
    if (line == "q") break;

    auto depth_img = sim.render(Eigen::Affine3d::Identity());

  }

  return 0;
}
