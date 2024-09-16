#include <gl_depth_sim/simulator/simulator_plugins.h>
#include <ros/ros.h>
#include <pluginlib/class_loader.h>

class Simulator
{
public:
  Simulator(const XmlRpc::XmlRpcValue &scene_update_plugin_config,
            const XmlRpc::XmlRpcValue &render_plugins_config,
            const double scene_update_rate = 30.0,
            const double render_rate = 30.0)
    : scene_update_plugin_loader_("gl_depth_sim", "gl_depth_sim::SceneUpdaterPlugin")
    , render_plugin_loader_("gl_depth_sim", "gl_depth_sim::RenderPlugin")
  {
    scene_update_plugin_ = scene_update_plugin_loader_.createInstance(
      static_cast<std::string>(scene_update_plugin_config["type"]));

    scene_update_plugin_->init(scene_update_plugin_config["params"]);

    // Load the render plugins
    for (int i = 0; i < render_plugins_config.size(); ++i)
    {
      XmlRpc::XmlRpcValue config = render_plugins_config[i];
      auto plugin = render_plugin_loader_.createInstance(static_cast<std::string>(config["type"]));
      plugin->init(config["params"]);
      render_plugins_.push_back(plugin);
    }

    // Create the scene
    scene_update_plugin_->createScene();

    // Create (but don't start) a timer for the updates
    ros::NodeHandle nh;
    scene_timer_ = nh.createTimer(ros::Rate(scene_update_rate), &Simulator::sceneTimerCallback, this, false, false);
    render_timer_ = nh.createTimer(ros::Rate(render_rate), &Simulator::renderTimerCallback, this, false, false);
  }

  void start()
  {
    scene_timer_.start();
    render_timer_.start();
  }

  void stop()
  {
    render_timer_.stop();
    scene_timer_.stop();
  }

  private:
  void sceneTimerCallback(const ros::TimerEvent &)
  {
    scene_update_plugin_->updateScene(/*sim_*/);
  }

  void renderTimerCallback(const ros::TimerEvent &)
  {
    for (auto &render_plugin : render_plugins_)
    {
      render_plugin->render(scene_update_plugin_->getScene());
    }
  }

  pluginlib::ClassLoader<gl_depth_sim::SceneUpdaterPlugin> scene_update_plugin_loader_;
  pluginlib::ClassLoader<gl_depth_sim::RenderPlugin> render_plugin_loader_;

  gl_depth_sim::SceneUpdaterPlugin::Ptr scene_update_plugin_;
  std::vector<gl_depth_sim::RenderPlugin::Ptr> render_plugins_;

  ros::Timer scene_timer_;
  ros::Timer render_timer_;
};

template<typename T>
T get(const ros::NodeHandle &nh, const std::string &key)
{
  T val;
  if (!nh.getParam(key, val))
  {
    throw std::runtime_error("Failed to get '" + key + "' parameter");
  }

  return val;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gl_depth_simulation");
  ros::NodeHandle pnh("~");

  sleep(3.0);

  try
  {
    // Load the plugin configurations
    XmlRpc::XmlRpcValue scene_update_plugin_config
      = get<XmlRpc::XmlRpcValue>(pnh, "scene_update_plugin");
    XmlRpc::XmlRpcValue render_plugins_config = get<XmlRpc::XmlRpcValue>(pnh, "render_plugins");

    // Load the publish rate
    double scene_update_rate = get<double>(pnh, "scene_update_rate");
    double render_update_rate = get<double>(pnh, "render_rate");

    // Create the simulation
    Simulator sim(scene_update_plugin_config, render_plugins_config, scene_update_rate, render_update_rate);

    // Start the simulator
    ROS_INFO_STREAM("Starting the simulator");
    sim.start();
    ros::spin();
  }
  catch (const std::exception &ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return -1;
  }

  return 0;
}

