#ifndef GL_DEPTH_SIM_PCL_INTERFACE_H
#define GL_DEPTH_SIM_PCL_INTERFACE_H

#include "gl_depth_sim/camera_properties.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gl_depth_sim
{

/**
 * @brief Projects the given depth image forward into space using the camera intrinsics from @e camera. The
 * resulting point cloud is organized (has width and height) and NOT dense. Invalid points are marked as NAN.
 */
void toPointCloudXYZ(const CameraProperties& camera, const DepthImage& depth, pcl::PointCloud<pcl::PointXYZ>& out);

}

#endif
