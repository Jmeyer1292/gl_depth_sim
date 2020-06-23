#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"

#include "gl_depth_sim/interfaces/pcl_interface.h"
#include "gl_depth_sim/interfaces/opencv_interface.h"

#include <iostream>

#include <pcl/io/pcd_io.h>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: ./depth_example <PATH_TO_MESH>\n";
    return 1;
  }

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

  auto mesh_ptr = gl_depth_sim::loadMesh(argv[1]);

  if (!mesh_ptr)
  {
    std::cerr << "Unable to load mesh from path: " << argv[1] << "\n";
    return 1;
  }

  sim.add("mesh_identifier", *mesh_ptr, Eigen::Isometry3d::Identity());

  std::cout << "Camera is looking down the positive x axis\n";
  std::cout << "Enter 'q' and press enter to quit\n";
  std::cout << "Input the camera positiion (x,y,z) with spaces in between and press enter:\n";
  std::string line;
  while (std::getline(std::cin, line))
  {
    if (line == "q") break;

    double x, y, z;

    std::istringstream iss (line);
    iss >> x >> y >> z;

    Eigen::Isometry3d pose; pose = Eigen::Translation3d(x,y,z);

    Eigen::Matrix3d m;
    m << 0, 0,  1,
         -1, 0, 0,
         0, -1, 0;
    pose.linear() = m;

    auto depth_img = sim.render(pose);

    cv::Mat img;
    gl_depth_sim::toCvImage16u(depth_img, img);
    cv::imwrite("img.png", img);

    pcl::PointCloud<pcl::PointXYZ> cloud;
    gl_depth_sim::toPointCloudXYZ(props, depth_img, cloud);
    pcl::io::savePCDFileBinary("cloud.pcd", cloud);
  }

  return 0;
}
