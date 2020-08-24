#ifndef GL_DEPTH_SIM_SIM_LASER_SCANNER_H
#define GL_DEPTH_SIM_SIM_LASER_SCANNER_H

#include <math.h>
#include <chrono>
#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h" // For converting to PCL cloud
#include "gl_depth_sim/renderable_mesh.h"
#include "gl_depth_sim/interfaces/opencv_interface.h"
#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"

#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

namespace gl_depth_sim
{
class SimLaserScanner {
public:
    SimLaserScanner (gl_depth_sim::CameraProperties);
    const gl_depth_sim::DepthImage render(Eigen::Isometry3d& camera_pose);
    void add(const gl_depth_sim::Mesh& mesh, const Eigen::Isometry3d& pose);

private:
    gl_depth_sim::SimDepthCamera camera_;
    gl_depth_sim::CameraProperties properties_;
};
}

#endif // GL_DEPTH_SIM_SIM_LASER_SCANNER_H
