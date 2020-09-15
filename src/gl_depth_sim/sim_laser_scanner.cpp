#include "gl_depth_sim/sim_laser_scanner.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"

#include <pcl/common/transforms.h>

namespace gl_depth_sim
{
SimLaserScanner::SimLaserScanner(const LaserScannerProperties &props)
  : camera_properties_(props.createCameraProperties())
  , camera_(camera_properties_)
{
}

void SimLaserScanner::add(const Mesh &mesh, const Eigen::Isometry3d &pose)
{
  camera_.add(mesh, pose);
}

const pcl::PointCloud<pcl::PointXYZ> SimLaserScanner::render(const Eigen::Isometry3d &scanner_pose)
{
  pcl::PointCloud<pcl::PointXYZ> scan;

  // Create a camera pose that the z-axis looks out in the scanner x-y plane
  Eigen::AngleAxisd camera_offset(M_PI_2, Eigen::Vector3d::UnitX());

  // Render the scene 3 times at 120 degree spacing, rotating about the y-axis
  for (std::size_t i = 0; i < 3; i++)
  {
    // Create a camera pose rotation
    double angle = i * (2.0 * M_PI / 3.0);
    Eigen::AngleAxisd rotation(angle, Eigen::Vector3d::UnitY());

    // Create a overall offset transform
    Eigen::Isometry3d offset(Eigen::Isometry3d::Identity());
    offset *= camera_offset * rotation;

    // Render the depth data
    DepthImage depth_data = camera_.render(scanner_pose * offset);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    toPointCloudXYZ(camera_properties_, depth_data, cloud);

    // Transform this cloud back into the original scanner frame
    pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    pcl::transformPointCloud(cloud, transformed_cloud, offset.matrix());

    // Append this scan to the output cloud
    scan += transformed_cloud;
  }

  return scan;
}

} // namespace gl_depth_sim

