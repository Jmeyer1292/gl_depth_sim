#include <ros/ros.h>
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

static void computeProperties(gl_depth_sim::CameraProperties& properties,
                         float z_near, float z_far, float resolution)
{
  properties.z_near = z_near;
  properties.z_far = z_far;
  float fov_x = 2*M_PI/3;
  properties.width = ceil(fov_x / resolution);
  properties.height  = 1; //TODO is there a calculation for this?
  properties.fx = properties.width / (2*tan(fov_x/2));
  properties.fy = properties.fx;
  properties.cx = properties.width / 2;
  properties.cy = properties.height / 2;

}

class SimLaserScanner {
    gl_depth_sim::SimDepthCamera camera_;
    gl_depth_sim::CameraProperties properties_;
  public:
    SimLaserScanner (gl_depth_sim::CameraProperties);
    pcl::PointCloud<pcl::PointXYZ> render(Eigen::Isometry3d& camera_pose,
            tf::TransformBroadcaster& broadcaster, ros::Publisher& cloud_pub);
    void add(const gl_depth_sim::Mesh& mesh, const Eigen::Isometry3d& pose);

};


SimLaserScanner::SimLaserScanner  (gl_depth_sim::CameraProperties props)
  : camera_(props), properties_(props) {}


void SimLaserScanner::add(const gl_depth_sim::Mesh& mesh, const Eigen::Isometry3d& pose){
  camera_.add(mesh, pose);
}


pcl::PointCloud<pcl::PointXYZ> SimLaserScanner::render(Eigen::Isometry3d&camera_pose,
                               tf::TransformBroadcaster& broadcaster, ros::Publisher& cloud_pub){
  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (int i=0; i<3; i++){
    const auto depth_data = camera_.render(camera_pose);
    gl_depth_sim::toPointCloudXYZ(properties_, depth_data, cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud.header.frame_id = "camera";
    cloud_pub.publish(cloud);
    tf::Transform camera_transform;
    tf::transformEigenToTF(camera_pose, camera_transform);
    tf::StampedTransform camera_stamped_transform (camera_transform, ros::Time::now(), "world", "camera");
    broadcaster.sendTransform(camera_stamped_transform);
    camera_pose.rotate(Eigen::AngleAxisd( 2*M_PI/3, Eigen::Vector3d::UnitY()));
  }
  return cloud;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "laser_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);
  tf::TransformBroadcaster broadcaster;
  std::string base_frame = "world";
  std::string camera_frame = "camera";
  std::string mesh_frame = "mesh";

  gl_depth_sim::CameraProperties props;
  float z_near, z_far, resolution;
  nh.getParam("/laser_node/z_near", z_near);
  nh.getParam("/laser_node/z_far", z_far);
  nh.getParam("/laser_node/resolution", resolution);
  computeProperties(props, z_near, z_far, resolution);

  SimLaserScanner laser(props);

  std::string mesh_path;
  nh.getParam("/laser_node/path", mesh_path);
  std::unique_ptr<gl_depth_sim::Mesh> mesh_ptr = gl_depth_sim::loadMesh(mesh_path);
  auto mesh_pose = Eigen::Isometry3d::Identity();
//  mesh_pose.translation() = Eigen::Vector3d(1, -1, 0);
  laser.add(*mesh_ptr, mesh_pose);

  /*
//  double cam_x, cam_y, cam_z;
//  nh.getParam("/laser_node/cam_x", cam_x);
//  nh.getParam("/laser_node/cam_y", cam_y);
//  nh.getParam("/laser_node/cam_z", cam_z);
//  Eigen::Vector3d camera_pos (cam_x,cam_y,cam_z);
//  auto camera_pose = lookat(camera_pos, Eigen::Vector3d(1,0,0), Eigen::Vector3d(0,0,1));
  */
  const auto start = std::chrono::steady_clock::now();
  while (ros::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();
    const static double radius = 2.0;
    Eigen::Vector3d camera_pos (radius * cos(dt/2),
                                radius * sin(dt/2),
                                2.0);

    auto camera_pose = lookat(camera_pos,  Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1));

    auto cloud = laser.render(camera_pose, broadcaster, cloud_pub);
    cloud.header.frame_id = camera_frame;
    cloud_pub.publish(cloud);
  }

  ros::waitForShutdown();
  return 0;
}
