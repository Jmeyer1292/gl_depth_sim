#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>

#include <opencv2/highgui/highgui.hpp>
#include "gl_depth_sim/interfaces/opencv_interface.h"
#include <pcl/io/pcd_io.h>

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

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ros_depth_sim_orbit");
  auto logger = node->get_logger();

  // Setup ROS interfaces
  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);

  tf2_ros::TransformBroadcaster broadcaster(*node);

  // Declare ROS parameters
  node->declare_parameter<std::string>("mesh");
  node->declare_parameter<std::string>("base_frame", "world");
  node->declare_parameter<std::string>("camera_frame", "camera");
  node->declare_parameter<double>("radius", 1.0);
  node->declare_parameter<double>("z", 1.0);
  node->declare_parameter<double>("focal_length", 550.0);
  node->declare_parameter<int>("width", 640);
  node->declare_parameter<int>("height", 480);

  // Load ROS parameters
  std::string mesh_path;
  if (!node->get_parameter("mesh", mesh_path))
  {
    RCLCPP_ERROR(logger, "User must set the 'mesh' parameter");
    rclcpp::shutdown();
    return 1;
  }

  std::string base_frame = node->get_parameter("base_frame").as_string();
  std::string camera_frame = node->get_parameter("camera_frame").as_string();

  double radius = node->get_parameter("radius").as_double();
  double z = node->get_parameter("z").as_double();
  double focal_length = node->get_parameter("focal_length").as_double();
  int width = node->get_parameter("width").as_int();
  int height = node->get_parameter("height").as_int();

  auto mesh_ptr = gl_depth_sim::loadMesh(mesh_path);
  if (!mesh_ptr)
  {
    RCLCPP_ERROR(logger, "Unable to load mesh from path: %s", mesh_path.c_str());
    rclcpp::shutdown();
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
  gl_depth_sim::SimDepthCamera sim(props);
  sim.add("mesh_identifier", *mesh_ptr, Eigen::Isometry3d::Identity());


  // State for FPS monitoring
  long frame_counter = 0;
  // In the main (rendering) thread, begin orbiting...
  const auto start = std::chrono::steady_clock::now();

  while (rclcpp::ok())
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    Eigen::Vector3d camera_pos (radius * cos(dt),
                                radius * sin(dt),
                                z);

    Eigen::Vector3d look_at (0,0,0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0,0,1));

    const auto depth_img = sim.render(pose);

    frame_counter++;

    if (frame_counter % 100 == 0)
    {
      std::cout << "FPS: " << frame_counter / dt << "\n";
    }

    // Step 1: Publish the cloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    gl_depth_sim::toPointCloudXYZ(props, depth_img, cloud);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = camera_frame;
    cloud_msg.header.stamp = node->now();
    cloud_pub->publish(cloud_msg);

    // Step 2: Publish the TF so we can see it in RViz
    auto transform = tf2::eigenToTransform(pose);
    transform.header.frame_id = base_frame;
    transform.header.stamp = node->now();
    transform.child_frame_id = camera_frame;
    broadcaster.sendTransform(transform);

    cv::Mat img;
    gl_depth_sim::toCvImage16u(depth_img, img);
    cv::imwrite("img.png", img);

    rclcpp::spin_some(node);
  }

  return 0;
}
