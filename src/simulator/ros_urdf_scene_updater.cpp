#include <gl_depth_sim/simulator/ros_urdf_scene_updater.h>
#include <gl_depth_sim/mesh_loader.h>

#include <parallel/algorithm>
#include <regex>
#include <ros/package.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/exceptions.h>
#include <urdf/model.h>

const static double TIMEOUT = 3.0;
const static std::string URDF_PARAM = "robot_description";

namespace
{
bool resolveURI(const std::string& in, const std::string& uri, std::string& out)
{
  std::regex expression(uri + "(\\w*)(\\/.*)");
  if (std::regex_match(in, expression))
  {
    std::smatch matches;
    std::regex_search(in, matches, expression);

    out = ros::package::getPath(matches[1].str());
    out += matches[2].str();

    return true;
  }

  return false;
}

Eigen::Isometry3d poseURDFToEigen(const urdf::Pose& pose)
{
  Eigen::Vector3d translation(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond rotation(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);

  Eigen::Isometry3d out(Eigen::Isometry3d::Identity());
  out.translate(translation);
  out.rotate(rotation);

  return out;
}

}  // namespace

namespace gl_depth_sim
{
ROSURDFSceneUpdaterPlugin::ROSURDFSceneUpdaterPlugin()
  : SceneUpdaterPlugin()
  , buffer_()
  , listener_(buffer_)
{
}

void ROSURDFSceneUpdaterPlugin::init(const XmlRpc::XmlRpcValue &)
{
}

void ROSURDFSceneUpdaterPlugin::createScene(/*gl_depth_sim::SimDepthCamera &sim_*/)
{
  urdf::Model model;
  if (!model.initParam(URDF_PARAM))
  {
    throw std::runtime_error("Failed to parse urdf file");
  }
  ROS_INFO("Successfully parsed urdf file");

  fixed_frame_ = model.getRoot()->name;

  // Iterate through all of the links and add the visual geometries
  for (auto it = model.links_.begin(); it != model.links_.end(); it++)
  {
    const auto& link = model.getLink(it->first);

    // Only include links with geometry
    if (link && link->visual_array.size() > 0)
    {
      // Create a container for all of the visuals defined in this link
      std::vector<std::unique_ptr<gl_depth_sim::Mesh>> visuals;
      visuals.reserve(link->visual_array.size());

      // Iterate over all of the visuals in this link
      for (std::size_t i = 0; i < link->visual_array.size(); ++i)
      {
        std::unique_ptr<gl_depth_sim::Mesh> mesh;
        switch (link->visual_array[i]->geometry->type)
        {
          case urdf::Geometry::MESH:
          {
            // Attempt to cast the visual pointer to a mesh
            const urdf::MeshConstSharedPtr tmp =
                urdf::dynamic_pointer_cast<const urdf::Mesh>(link->visual_array[i]->geometry);

            if (tmp)
            {
              // Get filepath and link name
              std::string filepath = tmp->filename;
              std::string link_name = link->name;  // This is also the name of the tf associated with this link

              // Exclude unsupported filetypes
              // dae files are still broken as of 8/21/18. The internal transforms are not properly imported by assimp
              // in gl_depth_sim
              if (filepath.substr(filepath.size() - 3) == "DAE" || filepath.substr(filepath.size() - 3) == "dae")
              {
                throw std::runtime_error("DAE files are currently unsupported");
              }

              // Load the object's mesh
              std::string mesh_filename;
              if (!resolveURI(filepath, "package://", mesh_filename))
              {
                if (!resolveURI(filepath, "file://", mesh_filename))
                {
                  mesh_filename = filepath;
                }
              }

              visuals.emplace_back(gl_depth_sim::loadMesh(mesh_filename));
            }

            break;
          }
          default:
            // TODO: add support for geometry primitives
            ROS_WARN_STREAM("Visual geometry other than meshes are not currently handled");
            break;
        }
      }

      // Create a single mesh from all of the vertices and indices of the visuals of this link
      std::unique_ptr<Mesh> mesh;
      {
        EigenAlignedVec<Eigen::Vector3f> vertices;
        std::vector<unsigned> indices;
        for (const auto& visual : visuals)
        {
          // Check that the visual was loaded correctly
          if (!visual)
            throw std::runtime_error("Failed to load visual mesh for link '" + link->name + "'");

          // Add the vertices directly
          vertices.insert(vertices.end(), visual->vertices().begin(), visual->vertices().end());

          // Offset this mesh's indices by the current size of the indices vector
          std::vector<unsigned> updated_mesh_indices(visual->indices());
          std::transform(updated_mesh_indices.begin(), updated_mesh_indices.end(),
                         updated_mesh_indices.begin(), [&indices](unsigned v) -> unsigned { return v + indices.size(); });

          indices.insert(indices.end(), updated_mesh_indices.begin(), updated_mesh_indices.end());
        }
        mesh = std::make_unique<gl_depth_sim::Mesh>(vertices, indices);
      }

      // Get the object's position relative to the fixed frame
      geometry_msgs::TransformStamped mesh_transform =
          buffer_.lookupTransform(fixed_frame_, link->name, ros::Time(0), ros::Duration(TIMEOUT));
      Eigen::Isometry3d pose = tf2::transformToEigen(mesh_transform);

      // Post-multiply the pose of the relative transform of the mesh points to its origin
      Eigen::Isometry3d relative_pose = poseURDFToEigen(link->visual->origin);
      relative_poses_.emplace(link->name, relative_pose);

      // Create the renderable object state
      gl_depth_sim::RenderableObjectState object;
      object.mesh = std::make_shared<gl_depth_sim::RenderableMesh>(*mesh);
      object.pose = pose * relative_pose;

      scene_.emplace(link->name, object);

      ROS_DEBUG_STREAM("Added mesh for link '" << link->name << "'");
    }
  }
}

void ROSURDFSceneUpdaterPlugin::updateScene()
{
  // Create a function that updates the position of each renderable object
  auto update_fn = [this](std::pair<const std::string, RenderableObjectState> &pair) -> void {
    // Look up the transform to the object
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = buffer_.lookupTransform(fixed_frame_,
                                          pair.first,
                                          ros::Time(0),
                                          ros::Duration(TIMEOUT));

      Eigen::Isometry3d pose = tf2::transformToEigen(transform);

      // Apply the relative pose of the visual geometry
      pair.second.pose = pose * relative_poses_.at(pair.first);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_ERROR_STREAM(ex.what());
    }
  };

  // Update the transforms of the meshes in the environment
  __gnu_parallel::for_each(scene_.begin(), scene_.end(), update_fn);
}


}  // namespace amsted_vision_processing

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(gl_depth_sim::ROSURDFSceneUpdaterPlugin, gl_depth_sim::SceneUpdaterPlugin);
