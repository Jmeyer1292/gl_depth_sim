#ifndef GL_DEPTH_SIM_SIMULATOR_ROS_SCENE_UPDATER_H
#define GL_DEPTH_SIM_SIMULATOR_ROS_SCENE_UPDATER_H

#include <gl_depth_sim/simulator/simulator_plugins.h>
#include <tf2_ros/transform_listener.h>
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace gl_depth_sim
{
/**
 * @brief Scene updater plugin which creates a scene from URDF and updates it using transform information from TF
 */
class ROSURDFSceneUpdaterPlugin : public SceneUpdaterPlugin
{
public:
  ROSURDFSceneUpdaterPlugin();

  virtual void init(const XmlRpc::XmlRpcValue &config) override;

  virtual void createScene() override;
  virtual void updateScene() override;

private:
  std::string fixed_frame_;

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;

  std::map<std::string, Eigen::Isometry3d> relative_poses_;
};

} // namespace gl_depth_sim

#endif // GL_DEPTH_SIM_SIMULATOR_ROS_SCENE_UPDATER_H
