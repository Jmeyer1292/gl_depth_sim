# GL Depth Simulator
This library simulates an organized depth camera and laser scanner by reading back a depth buffer after calling OpenGL to render a scene.
I tried to write this package so that the core library had only a couple of dependencies and could stand on its own. The primary use case, however, is simulating depth data and laser scan data WITH hardware acceleration and WITHOUT needing to use Gazebo for ROS robotics development.

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
## Depth Camera Example
![Stanford Dragon](docs/depth.gif)
This scene shows the classic Stanford Dragon seen by a depth camera orbiting around the model. Rviz is used to display the clouds. This was taken directly from the ros example in the src/ directory. While the GIF is slow, the scan data is published at hundreds of frames per second.

You can run this example in ROS by building this package in your workspace and running:
```
rosrun gl_depth_sim ros_example _mesh:=<PATH_TO_YOUR_MESH>
```

You can also set the `_z` and `_radius` parameters.

## Laser Scanner Example
![Stanford Dragon](docs/laser.mp4)
This library allows modeling a laser scanner and outputting scan data including a pointcloud of the scanner data
and the distances to points in the pointcloud. The SimLaserScanner class takes three input parameters all stored
in the LaserScannerProperties struct. These are the clipping distances (near and far) and the angular resolution.
The field of view is fixed at 120 degrees, meaning three rotations are required for a 360 degree scan. The height
is also fixed at 3 rather than 1 to prevent runtime errors. Only the middle row is used for analysis, like a
real laser scanner which is one-dimensional. All the other properties of the a laser scanner can be derived from
these values. This library includes the SimLaserScanner class, as well as an example node which demonstrates its
use on the Stanford Dragon.

The example node shows the Stanford Dragon seen by a laser scanner looking down at the dragon and scanning the
pointcloud. Rviz is used to display the clouds. This was taken directly from the ros example in the src/ directory.

You can run this example in ROS by building this package in your workspace and running:
```
rosrun gl_depth_sim laser_example.launch
```

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
  sim.add(*mesh_ptr, Eigen::Isometry3d::Identity());

  // Define a camera location from which you want to "take a picture"
  // Uses ROS conventions for optical frames (+Z down the camera field of view, Y down the image)
  Eigen::Isometry3d camera_pose = Eigen::Isometry3d::Identity();
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
