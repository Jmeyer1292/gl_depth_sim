#include <ros/ros.h>
#include <math.h>
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

#include <opencv2/highgui/highgui.hpp>
#include "gl_depth_sim/interfaces/opencv_interface.h"
#include <pcl/io/pcd_io.h>

#include <chrono>

#include <iostream>
#include <Eigen/Dense>


static Eigen::Isometry3d lookat(const Eigen::Vector3d& origin,
                                const Eigen::Vector3d& eye,
                                const Eigen::Vector3d& up)
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

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1, true);
  tf::TransformBroadcaster broadcaster;
  std::string base_frame = "world";
  std::string camera_frame = "camera";
  std::string mesh_frame = "mesh";

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

  std::string mesh_path;
  nh.getParam("/laser_node/path", mesh_path);
  std::unique_ptr<gl_depth_sim::Mesh> mesh_ptr = gl_depth_sim::loadMesh(mesh_path);
  auto mesh_pose = Eigen::Isometry3d::Identity();
//  mesh_pose.translation() = Eigen::Vector3d(1, -1, 0);
  sim.add("mesh_identifier", *mesh_ptr, mesh_pose);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = camera_frame;

  Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
//  Eigen::Vector3d camera_pos (0,0,0);
//  auto camera_pose = lookat(camera_pos, Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));
//  auto camera_pose = lookat(camera_pos, Eigen::Vector3d(1,-1,0), Eigen::Vector3d(0,0,1));
  while (ros::ok())
  {
    const auto depth_data = sim.render(camera_pose);
    gl_depth_sim::toPointCloudXYZ(props, depth_data, cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud_pub.publish(cloud);
    tf::Transform camera_transform;
    tf::transformEigenToTF(camera_pose, camera_transform);
    tf::StampedTransform camera_stamped_transform (camera_transform, ros::Time::now(), base_frame, camera_frame);
    broadcaster.sendTransform(camera_stamped_transform);
    camera_pose.rotate(Eigen::AngleAxisd(2 * M_PI / 3, Eigen::Vector3d::UnitY()));
    ros::Duration(1).sleep();
  }

  return 0;
}
