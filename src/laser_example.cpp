#include <gl_depth_sim/sim_laser_scanner.h>
#include <gl_depth_sim/mesh_loader.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/io/vtk_lib_io.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>
#include <visualization_msgs/Marker.h>

template<typename T>
bool get(const ros::NodeHandle &nh, const std::string &key, T &val)
{
  if (!nh.getParam(key, val))
  {
    ROS_ERROR_STREAM("Failed to get '" << key << "' parameter");
    return false;
  }
  return true;
}

visualization_msgs::Marker createMeshMarker(const Eigen::Isometry3d &pose,
                                            const std::string &frame,
                                            const std::string &mesh_resource)
{
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame;
  marker.id = 0;
  marker.ns = "mesh";
  marker.action = visualization_msgs::Marker::ADD;

  marker.type = visualization_msgs::Marker::MESH_RESOURCE;

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
  ros::init(argc, argv, "laser_node");
  ros::NodeHandle nh, pnh("~");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Publisher cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);
  tf2_ros::TransformBroadcaster broadcaster;
  std::string base_frame = "world";
  std::string camera_frame = "camera";

  std::string mesh_filename;
  if (!get(pnh, "mesh_filename", mesh_filename))
    return -1;

  // Get the laser scanner properties
  gl_depth_sim::LaserScannerProperties laser_scan_props;
  if (!get(pnh, "min_range", laser_scan_props.min_range)
      || !get(pnh, "max_range", laser_scan_props.max_range)
      || !get(pnh, "angular_resolution", laser_scan_props.angular_resolution))
  {
    return -1;
  }

  // Create the laser scanner
  gl_depth_sim::SimLaserScanner laser(laser_scan_props);

  // Add the mesh
  std::unique_ptr<gl_depth_sim::Mesh> mesh_ptr = gl_depth_sim::loadMesh(mesh_filename);
  Eigen::Isometry3d mesh_pose(Eigen::Isometry3d::Identity());
  laser.add(*mesh_ptr, mesh_pose);

  // Publish a message with the mesh for visualization
  ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("object", 1, true);
  visualization_msgs::Marker marker = createMeshMarker(mesh_pose, base_frame, mesh_filename);
  pub.publish(marker);

  // Sweep the laser scanner back and forth across the surface of a part in the world y-axis direction
  //
  const double sweep_distance = 0.8;
  Eigen::Isometry3d nominal_scanner_pose = Eigen::Isometry3d::Identity();
  nominal_scanner_pose.translate(Eigen::Vector3d(0.0, 0.0, 2.0));
  nominal_scanner_pose.rotate(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));

  std::size_t counter = 0;
  Eigen::Isometry3d scanner_pose(nominal_scanner_pose);
  while (ros::ok())
  {
    pcl::PointCloud<pcl::PointXYZ> scan = laser.render(scanner_pose);

    // Publish the scan cloud
    scan.header.frame_id = camera_frame;
    pcl_conversions::toPCL(ros::Time::now(), scan.header.stamp);
    cloud_pub.publish(scan);

    // Wait
    ros::Duration(0.005).sleep();

    // Update the transform
    ++counter;
    double z = std::sin(static_cast<double>(counter) / 180.0) * sweep_distance;
    scanner_pose = nominal_scanner_pose * Eigen::Translation3d(Eigen::Vector3d(0.0, 0.0, z));

    geometry_msgs::TransformStamped transform = tf2::eigenToTransform(scanner_pose);
    transform.header.frame_id = base_frame;
    transform.header.stamp = ros::Time::now();
    transform.child_frame_id = camera_frame;
    broadcaster.sendTransform(transform);
  }

  ros::waitForShutdown();
  return 0;
}
