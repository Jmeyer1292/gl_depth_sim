#include <ros/ros.h>
#include <gl_depth_sim/mesh_loader.h>
#include <gl_depth_sim/sim_depth_camera.h>

#include <tf/transform_listener.h>

class Simulation
{
public:
};

std::vector<std::pair<Eigen::Affine3d, std::unique_ptr<gl_depth_sim::Mesh>>>
loadSceneFromParams(ros::NodeHandle& nh, const std::string& ns)
{
  return {};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gl_depth_camera_sim");
  ros::NodeHandle nh;

  // ROS Scene Loader
  auto scene = loadSceneFromParams(nh, "scene");


  ros::spin();
  return 0;
}
