#include <gl_depth_sim/simulator/laser_scanner_plugin.h>

#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

const static double TIMEOUT = 3.0;
static std::size_t SEQ = 0;

namespace gl_depth_sim
{

LaserScannerPlugin::LaserScannerPlugin()
  : listener_(buffer_)
{

}

void LaserScannerPlugin::init(const XmlRpc::XmlRpcValue& config)
{
  fixed_frame_ = static_cast<std::string>(config["fixed_frame"]);
  scanner_frame_ = static_cast<std::string>(config["camera_frame"]);

  props_.max_range = static_cast<double>(config["max_range"]);
  props_.min_range = static_cast<double>(config["min_range"]);
  props_.angular_resolution = static_cast<double>(config["angular_resolution"]);

  sim_ = std::make_unique<SimLaserScanner>(props_);

  std::string topic_name = static_cast<std::string>(config["topic"]);
  ros::NodeHandle nh;
  pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);
}

void LaserScannerPlugin::render(const std::map<std::string, RenderableObjectState>& scene)
{
  // Get camera tranform
  geometry_msgs::TransformStamped scanner_transform
      = buffer_.lookupTransform(fixed_frame_, scanner_frame_, ros::Time(0), ros::Duration(TIMEOUT));

  Eigen::Isometry3d camera_pose = tf2::transformToEigen(scanner_transform);

  // Render the depth image
  pcl::PointCloud<pcl::PointXYZ> cloud = sim_->render(camera_pose, scene);

  // Publish the cloud
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = scanner_frame_;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = SEQ++;
  pub_.publish(msg);
}

}  // namespace gl_depth_sim

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gl_depth_sim::LaserScannerPlugin, gl_depth_sim::RenderPlugin);
