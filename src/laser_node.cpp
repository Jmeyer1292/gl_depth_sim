#include <ros/ros.h>
#include "gl_depth_sim/sim_depth_camera.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "gl_depth_sim/interfaces/pcl_interface.h"
#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h" // For converting to PCL cloud

#include <chrono>



static Eigen::Isometry3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Isometry3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

class SimLaserScanner {
    gl_depth_sim::SimDepthCamera camera_;
    float z_near_, z_far_;
  public:
    SimLaserScanner (gl_depth_sim::CameraProperties, float, float);
    pcl::PointCloud<pcl::PointXYZ> render();

};



SimLaserScanner::SimLaserScanner (gl_depth_sim::CameraProperties props, float z_near, float z_far)
    : camera_(props), z_near_(z_near), z_far_(z_far) {}


pcl::PointCloud<pcl::PointXYZ> SimLaserScanner::render(){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  return cloud;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_node");
  ros::NodeHandle nh;

  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1, true);
  tf::TransformBroadcaster broadcaster;
  std::string base_frame = "world";
  std::string camera_frame = "camera";

  gl_depth_sim::CameraProperties props;
  nh.getParam("/laser_node/width", props.width);
  nh.getParam("/laser_node/height", props.height);
  nh.getParam("/laser_node/fx", props.fx);
  nh.getParam("/laser_node/fy", props.fy);
  nh.getParam("/laser_node/cx", props.cx);
  nh.getParam("/laser_node/cy", props.cy);
  nh.getParam("/laser_node/z_near", props.z_near);
  nh.getParam("/laser_node/z_far", props.z_far);


  gl_depth_sim::SimDepthCamera sim (props);

  std::string mesh_path = "/home/cwolfe/a5_ws/src/A5/a5_support/meshes/a5/visual/wheel_hub.stl";
  std::unique_ptr<gl_depth_sim::Mesh> mesh_ptr = gl_depth_sim::loadMesh(mesh_path);
  sim.add(*mesh_ptr, Eigen::Isometry3d::Identity());

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = camera_frame;

  const auto start = std::chrono::steady_clock::now();

  while (ros::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

//    Eigen::Vector3d camera_pos = Eigen::Vector3d(1,0,1);
    Eigen::Vector3d camera_pos (cos(dt), sin(dt), 1.0);
    Eigen::Vector3d look_at (0,0,0);
    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0,0,1));
//    camera_pos.translation() = Eigen::Vector3d(-1, 0, 0);

    gl_depth_sim::DepthImage depth_data = sim.render(pose);

    gl_depth_sim::toPointCloudXYZ(props, depth_data, cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud_pub.publish(cloud);

    // Step 2: Publish the TF so we can see it in RViz
    tf::Transform transform;
    tf::transformEigenToTF(pose, transform);
    tf::StampedTransform stamped_transform (transform, ros::Time::now(), base_frame, camera_frame);
    broadcaster.sendTransform(stamped_transform);


  //  SimLaserScanner laser (props, props.z_near, props.z_far);
  //  laser.render();

    ros::spinOnce();
  }

  return 0;
}
