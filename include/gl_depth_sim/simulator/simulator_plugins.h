#ifndef GL_DEPTH_SIM_SIMULATOR_SIMULATOR_PLUGINS_H
#define GL_DEPTH_SIM_SIMULATOR_SIMULATOR_PLUGINS_H

#include <gl_depth_sim/sim_depth_camera.h>

#include <xmlrpcpp/XmlRpcValue.h>
#include <boost/shared_ptr.hpp>

namespace gl_depth_sim
{
/**
 * @brief Base class plugin to create and update a renderable scene
 */
class SceneUpdaterPlugin
{
public:
  using Ptr = boost::shared_ptr<SceneUpdaterPlugin>;
  SceneUpdaterPlugin() = default;
  virtual ~SceneUpdaterPlugin() = default;

  /**
   * @brief Initializes the plugin from an XMLRPC configuration
   * @param config
   */
  virtual void init(const XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief Creates a scene with renderable objects
   */
  virtual void createScene() = 0;

  /**
   * @brief Updates the location of renderable objects within the scene
   */
  virtual void updateScene() = 0;

  /**
   * @brief Returns the representation of the scene
   * @return
   */
  inline const std::map<std::string, RenderableObjectState>& getScene() const
  {
    return scene_;
  }

protected:
  std::map<std::string, RenderableObjectState> scene_;
};

/**
 * @brief Base class plugin for rendering a scene
 */
class RenderPlugin
{
public:
  using Ptr = boost::shared_ptr<RenderPlugin>;
  RenderPlugin() = default;
  virtual ~RenderPlugin() = default;

  /**
   * @brief Initializes the plugin with an XMLRPC configuration
   * @param config
   */
  virtual void init(const XmlRpc::XmlRpcValue& config) = 0;

  /**
   * @brief Renders the input scene
   * @param scene
   */
  virtual void render(const std::map<std::string, RenderableObjectState>& scene) = 0;
};

}  // namespace gl_depth_sim

#endif  // GL_DEPTH_SIM_SIMULATOR_SIMULATOR_PLUGINS_H
