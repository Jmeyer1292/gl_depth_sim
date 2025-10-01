#include <gl_depth_sim/sim_laser_scanner.h>
#include <gl_depth_sim/mesh_loader.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/vtk_lib_io.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <visualization_msgs/msg/marker.hpp>

visualization_msgs::msg::Marker createMeshMarker(const Eigen::Isometry3d &pose,
                                                 const std::string &frame,
                                                 const std::string &mesh_resource)
{
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = frame;
  marker.id = 0;
  marker.ns = "mesh";
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

  // Check if the mesh resource does not use the "file://" or "package://" URI
  if (mesh_resource.find("file://") == std::string::npos
      && mesh_resource.find("package://") == std::string::npos)
  {
    // Assume that the provided mesh resource is a fully specified file path; pre-pend the "file://" URI
    marker.mesh_resource = "file://" + mesh_resource;
  }
  else
  {
    // The input mesh resource already has the correct URI specification
    marker.mesh_resource = mesh_resource;
  }

  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;

  marker.color.a = 1.0;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.pose = tf2::toMsg(pose);

  return marker;
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("laser_node");
  auto logger = node->get_logger();

  // Create single-threaded executor and spin in a separate thread
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  auto cloud_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud", 1);
  auto broadcaster = tf2_ros::TransformBroadcaster(*node);

  // Declare parameters
  node->declare_parameter<std::string>("base_frame", "world");
  node->declare_parameter<std::string>("camera_frame", "camera");
  node->declare_parameter<std::string>("mesh_filename");
  node->declare_parameter<double>("min_range");
  node->declare_parameter<double>("max_range");
  node->declare_parameter<double>("angular_resolution");

  // Get parameters
  std::string base_frame, camera_frame, mesh_filename;
  node->get_parameter("base_frame", base_frame);
  node->get_parameter("camera_frame", camera_frame);

  if (!node->get_parameter("mesh_filename", mesh_filename))
  {
    RCLCPP_ERROR(logger, "Parameter 'mesh_filename' is required.");
    rclcpp::shutdown();
    return -1;
  }

  // Get the laser scanner properties
  gl_depth_sim::LaserScannerProperties laser_scan_props;
  if (!node->get_parameter("min_range", laser_scan_props.min_range)
      || !node->get_parameter("max_range", laser_scan_props.max_range)
      || !node->get_parameter("angular_resolution", laser_scan_props.angular_resolution))
  {
    rclcpp::shutdown();
    return -1;
  }

  // Create the laser scanner
  gl_depth_sim::SimLaserScanner laser(laser_scan_props);

  // Add the mesh
  std::unique_ptr<gl_depth_sim::Mesh> mesh_ptr = gl_depth_sim::loadMesh(mesh_filename);
  Eigen::Isometry3d mesh_pose(Eigen::Isometry3d::Identity());
  laser.add(*mesh_ptr, mesh_pose);

  // Publish a message with the mesh for visualization
  auto pub = node->create_publisher<visualization_msgs::msg::Marker>("object", rclcpp::QoS(1).transient_local());
  auto marker = createMeshMarker(mesh_pose, base_frame, mesh_filename);
  pub->publish(marker);

  // Sweep the laser scanner back and forth across the surface of a part in the world y-axis direction
  //
  const double sweep_distance = 1.0;
  Eigen::Isometry3d nominal_scanner_pose = Eigen::Isometry3d::Identity();
  nominal_scanner_pose.translate(Eigen::Vector3d(0.0, 0.0, 2.0));
  nominal_scanner_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));

  std::size_t counter = 0;
  Eigen::Isometry3d scanner_pose(nominal_scanner_pose);
  while (rclcpp::ok())
  {
    pcl::PointCloud<pcl::PointXYZ> scan = laser.render(scanner_pose);

    // Publish the scan cloud
    sensor_msgs::msg::PointCloud2 scan_msg;
    pcl::toROSMsg(scan, scan_msg);
    scan_msg.header.frame_id = camera_frame;
    scan_msg.header.stamp = node->now();
    cloud_pub->publish(scan_msg);

    // Wait
    rclcpp::sleep_for(std::chrono::milliseconds(5));

    // Update the transform
    ++counter;
    double z = std::sin(static_cast<double>(counter) / 180.0) * sweep_distance;
    scanner_pose = nominal_scanner_pose * Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, z));

    auto transform = tf2::eigenToTransform(scanner_pose);
    transform.header.frame_id = base_frame;
    transform.header.stamp = node->now();
    transform.child_frame_id = camera_frame;
    broadcaster.sendTransform(transform);
  }

  executor.cancel();
  spin_thread.join();

  // Reset node before shutdown to avoid segfault (see ROS 2 issues)
  node.reset();

  rclcpp::shutdown();
  return 0;
}
