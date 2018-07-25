#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"
#include  "gl_depth_sim/interfaces/noise_model.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <chrono>

static Eigen::Affine3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Affine3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ros_depth_sim_orbit");
  ros::NodeHandle nh, pnh ("~");

  // Setup ROS interfaces
  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1, true);

  tf::TransformBroadcaster broadcaster;

  // Load ROS parameters
  std::string mesh_path;
  if (!pnh.getParam("mesh", mesh_path))
  {
    ROS_ERROR_STREAM("User must set the 'mesh' private parameter");
    return 1;
  }

  std::string base_frame = pnh.param<std::string>("base_frame", "world");
  std::string camera_frame = pnh.param<std::string>("camera_frame", "camera");

  double radius = pnh.param<double>("radius", 1.0);
  double z = pnh.param<double>("z", 1.0);

  double focal_length = pnh.param<double>("focal_length", 550.0);
  int width = pnh.param<int>("width", 640);
  int height = pnh.param<int>("height", 480);

  auto mesh_ptr = gl_depth_sim::loadMesh(mesh_path);

  if (!mesh_ptr)
  {
    ROS_ERROR_STREAM("Unable to load mesh from path: " << mesh_path);
    return 1;
  }

  gl_depth_sim::CameraProperties props;
  props.width = width;
  props.height = height;
  props.fx = focal_length;
  props.fy = focal_length;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25;
  props.z_far = 10.0f;

  // Create the simulation
  gl_depth_sim::SimDepthCamera sim (props);
  sim.add(*mesh_ptr, Eigen::Affine3d::Identity());


  // State for FPS monitoring
  long frame_counter = 0;
  // In the main (rendering) thread, begin orbiting...
  const auto start = std::chrono::steady_clock::now();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.header.frame_id = camera_frame;

  while (ros::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    Eigen::Vector3d camera_pos (radius * cos(dt),
                                radius * sin(dt),
                                z);

    Eigen::Vector3d look_at (0,0,0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0,0,1));

    auto depth_img = sim.render(pose);

    depth_img.data  = gl_depth_sim::noise(depth_img.data);

    frame_counter++;

    if (frame_counter % 100 == 0)
    {
      std::cout << "FPS: " << frame_counter / dt << "\n";
    }

    // Step 1: Publish the cloud
    gl_depth_sim::toPointCloudXYZ(props, depth_img, cloud);
    pcl_conversions::toPCL(ros::Time::now(), cloud.header.stamp);
    cloud_pub.publish(cloud);

    // Step 2: Publish the TF so we can see it in RViz
    tf::Transform transform;
    tf::transformEigenToTF(pose, transform);
    tf::StampedTransform stamped_transform (transform, ros::Time::now(), base_frame, camera_frame);
    broadcaster.sendTransform(stamped_transform);

    ros::spinOnce();
  }

  return 0;
}
