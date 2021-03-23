#include <gl_depth_sim/simulator/depth_camera_plugin.h>
#include <gl_depth_sim/interfaces/pcl_interface.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>

const static double TIMEOUT = 3.0;
static std::size_t SEQ = 0;

namespace gl_depth_sim
{
DepthCameraPlugin::DepthCameraPlugin()
  : RenderPlugin()
  , listener_(buffer_)
{
}

void DepthCameraPlugin::init(const XmlRpc::XmlRpcValue &config)
{
  fixed_frame_ = static_cast<std::string>(config["fixed_frame"]);
  camera_frame_ = static_cast<std::string>(config["camera_frame"]);

  props_.width = static_cast<int>(config["width"]);
  props_.height = static_cast<int>(config["width"]);
  props_.z_near = static_cast<double>(config["z_near"]);
  props_.z_far = static_cast<double>(config["z_far"]);
  props_.fx = static_cast<double>(config["fx"]);
  props_.fy = static_cast<double>(config["fy"]);
  props_.cx = static_cast<double>(config["cx"]);
  props_.cy = static_cast<double>(config["cy"]);

  sim_ = std::make_unique<SimDepthCamera>(props_);

  std::string topic_name = static_cast<std::string>(config["topic"]);
  ros::NodeHandle nh;
  pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 1, true);
}

void DepthCameraPlugin::render(const std::map<std::string, RenderableObjectState> &scene)
{
  // Get camera tranform
  geometry_msgs::TransformStamped camera_transform
    = buffer_.lookupTransform(fixed_frame_, camera_frame_, ros::Time(0), ros::Duration(TIMEOUT));

  Eigen::Isometry3d camera_pose = tf2::transformToEigen(camera_transform);

  // Render the depth image
  gl_depth_sim::DepthImage depth_img = sim_->render(camera_pose, scene);

  // Convert the depth image to point cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  gl_depth_sim::toPointCloudXYZ(props_, depth_img, cloud);

  // Publish the cloud
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = camera_frame_;
  msg.header.stamp = ros::Time::now();
  msg.header.seq = SEQ++;
  pub_.publish(msg);
}

} // namespace gl_depth_sim

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gl_depth_sim::DepthCameraPlugin, gl_depth_sim::RenderPlugin);
