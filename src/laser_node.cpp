#include <ros/ros.h>
#include <gl_depth_sim/simlaserscanner.h>

#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h" // For converting to PCL cloud


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_node");
  ros::NodeHandle nh;

  gl_depth_sim::CameraProperties props;
  props.width = 640; // In pixels
  props.height = 480;
  props.fx = 550.0f; // In pixels
  props.fy = 550.0f;
  props.cx = props.width / 2; // In pixels
  props.cy = props.height / 2;
  props.z_near = 0.25f; // In "world units"
  props.z_far = 10.0f;

  gl_depth_sim::SimDepthCamera sim (props);

//  SimLaserScanner laser (sim);

  ROS_INFO("Hello world!");
}

