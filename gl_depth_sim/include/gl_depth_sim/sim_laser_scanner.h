#ifndef GL_DEPTH_SIM_SIM_LASER_SCANNER_H
#define GL_DEPTH_SIM_SIM_LASER_SCANNER_H

#include <gl_depth_sim/camera_properties.h>
#include <gl_depth_sim/sim_depth_camera.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gl_depth_sim
{
struct LaserScannerProperties
{
  LaserScannerProperties() = default;
  LaserScannerProperties(const double min_range_, const double max_range_,
                         const double angular_resolution_)
    : min_range(min_range_)
    , max_range(max_range_)
    , angular_resolution(angular_resolution_)
  {
  }

  /** @brief Minimum range (m) */
  double min_range = 0.01;
  /** @brief Maximum range (m) */
  double max_range = 10.0;
  /** @brief Angular resolution (rad) */
  double angular_resolution = 0.001;

  /**
   * @brief Creates a set of depth simulator camera properties for this laser scanner
   * @return
   */
  CameraProperties createCameraProperties() const
  {
    const double fov_x = 2.0 * M_PI / 3.0;

    CameraProperties properties;
    properties.z_near = min_range;
    properties.z_far = max_range;
    properties.width = std::ceil(fov_x / angular_resolution);
    properties.height = 1;
    properties.fx = properties.width / (2.0 * std::tan(fov_x / 2.0));
    properties.fy = properties.fx;
    properties.cx = properties.width / 2.0;
    properties.cy = properties.height / 2.0;

    return properties;
  }
};

class SimLaserScanner
{
public:
  SimLaserScanner(const LaserScannerProperties& properties);

  /**
   * @brief Renders a laser scan from the input pose
   * @param scanner_pose
   * @return
   */
  const pcl::PointCloud<pcl::PointXYZ> render(const Eigen::Isometry3d& scanner_pose);

  /**
   * @brief Adds a mesh to the renderable environment
   * @param mesh
   * @param pose
   */
  void add(const Mesh& mesh, const Eigen::Isometry3d& pose);

private:
  const CameraProperties camera_properties_;
  gl_depth_sim::SimDepthCamera camera_;
};

} // namespace gl_depth_sim

#endif // GL_DEPTH_SIM_SIM_LASER_SCANNER_H
