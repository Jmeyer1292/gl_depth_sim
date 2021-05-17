#include "gl_depth_sim/interfaces/pcl_interface.h"

void gl_depth_sim::toPointCloudXYZ(const gl_depth_sim::CameraProperties& camera, const gl_depth_sim::DepthImage& depth,
                                   pcl::PointCloud<pcl::PointXYZ>& out)
{
  out.width = depth.cols;
  out.height = depth.rows;
  out.resize(out.width * out.height);
  out.is_dense = false;

  for (int i = 0; i < depth.rows; ++i)
  {
    for (int j = 0; j < depth.cols; ++j)
    {
      const float distance = depth.distance(i, j);
      pcl::PointXYZ& pt = out(j, i);

      if (distance != 0.0f)
      {
        pt.z = distance;
        pt.x = (j - camera.cx) * distance / camera.fx;
        pt.y = (i - camera.cy) * distance / camera.fy;
      }
      else
      {
        pt.z = pt.x = pt.y = std::numeric_limits<float>::quiet_NaN();
      }
    }
  } // end of loop
}
