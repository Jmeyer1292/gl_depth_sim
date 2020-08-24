#include "gl_depth_sim/sim_laser_scanner.h"


gl_depth_sim::SimLaserScanner::SimLaserScanner  (gl_depth_sim::CameraProperties props)
  : camera_(props), properties_(props) {std::cout << "IN CONSTRUCTOR";}


void gl_depth_sim::SimLaserScanner::add(const gl_depth_sim::Mesh& mesh, const Eigen::Isometry3d& pose){
  camera_.add(mesh, pose);
}


const gl_depth_sim::DepthImage gl_depth_sim::SimLaserScanner::render(Eigen::Isometry3d&camera_pose){
//  pcl::PointCloud<pcl::PointXYZ> cloud;
//  for (int i=0; i<3; i++){
    const gl_depth_sim::DepthImage depth_data = camera_.render(camera_pose);
//    camera_pose.rotate(Eigen::AngleAxisd( 2*M_PI/3, Eigen::Vector3d::UnitY()));
//  }
  return depth_data;
}
