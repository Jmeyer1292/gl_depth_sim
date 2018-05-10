# GL Depth Simulator
This library simulates an organized depth camera by reading back a depth buffer after calling OpenGL to render a scene.
I tried to write this package so that the core library had only a couple of dependencies and could stand on its own. The primary use case, however, is simulating depth data WITH hardware acceleration and WITHOUT needing to use Gazebo for ROS robotics development.

## Installation
### Dependencies
The core library requires:
 - GLFW (for GL context creation/windowing)
 - Eigen (linear algebra)
 - Assimp (for model loading in examples)

I provide extensions for using the resulting depth data (and thus have dependencies) with:
 - OpenCV (cv::Mat in both floating point and unsigned-16 formats)
 -  PCL (PointCloud\<PointXYZ>)


If you have ROS, you have everything but GLFW already. To get running:
```
sudo apt install libglfw3-dev
```
## Example
[ TODO IMAGE HERE ]

## Usage
To use in a ROS context,  add a catkin dependency on `gl_depth_sim` and follow the idea of the following example:
```c++
#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/pcl_interface.h" // For converting to PCL cloud

int main(int argc, char** argv)
{
  // Create a camera properties structure that defines the intrinsic
  // properties of the camera
  gl_depth_sim::CameraProperties props;
  props.width = 640; // In pixels
  props.height = 480;
  props.fx = 550.0f; // In pixels
  props.fy = 550.0f;
  props.cx = props.width / 2; // In pixels
  props.cy = props.height / 2;
  props.z_near = 0.25f; // In "world units"
  props.z_far = 10.0f;

  // Create the depth camera itself
  gl_depth_sim::SimDepthCamera sim (props);

  // Use convienence function to load mesh from a file. Returns nullptr on failure.
  std::unique_ptr<gl_depth_sim::Mesh> mesh_ptr = gl_depth_sim::loadMesh(argv[1]);

  // Adds mesh to the renderable scene at the origin
  sim.add(*mesh_ptr, Eigen::Affine3d::Identity());

  // Define a camera location from which you want to "take a picture"
  // Uses ROS conventions for optical frames (+Z down the camera field of view, Y down the image)
  Eigen::Affine3d camera_pose = Eigen::Affine3d::Identity();
  camera_pose.translation() = Eigen::Vector3d(-1, 0, 0);

  // Render the image and download the depth data
  gl_depth_sim::DepthImage depth_data = sim.render(camera_pose);

  // Lastly, convert it to a point cloud and do something with it...
  pcl::PointCloud<pcl::PointXYZ> cloud;
  gl_depth_sim::toPointCloudXYZ(props, depth_data, cloud);

  // Your code here...
```

## Issues
 1. Needs better scene management. Add, remove, move, etc...
 2. Needs qualification for depth accuracy. GPU depth buffers are fairly limited and have precision problems at distance. There are techniques to manage this and I'd like to add them
 3. I'd like to support color image generation too. It shouldn't be too bad but we need to have meshes with texture.
