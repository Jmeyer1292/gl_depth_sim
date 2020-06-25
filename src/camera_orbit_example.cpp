#include "gl_depth_sim/sim_depth_camera.h"
#include "gl_depth_sim/mesh_loader.h"
#include "gl_depth_sim/interfaces/opencv_interface.h"
#include "gl_depth_sim/interfaces/pcl_interface.h"

#include <pcl/io/pcd_io.h> // For saving files for PCL
#include <opencv2/highgui/highgui.hpp> // For saving opencv images

#include <chrono>
#include <thread>
#include <atomic>

static Eigen::Isometry3d lookat(const Eigen::Vector3d& origin, const Eigen::Vector3d& eye, const Eigen::Vector3d& up)
{
  Eigen::Vector3d z = (eye - origin).normalized();
  Eigen::Vector3d x = z.cross(up).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();

  auto p = Eigen::Isometry3d::Identity();
  p.translation() = origin;
  p.matrix().col(0).head<3>() = x;
  p.matrix().col(1).head<3>() = y;
  p.matrix().col(2).head<3>() = z;
  return p;
}

int main(int argc, char** argv)
{
  if (argc != 2)
  {
    std::cerr << "Usage: ./orbit_example <PATH_TO_MESH>\n";
    return 1;
  }

  auto mesh_ptr = gl_depth_sim::loadMesh(argv[1]);

  if (!mesh_ptr)
  {
    std::cerr << "Unable to load mesh from path: " << argv[1] << "\n";
    return 1;
  }

  gl_depth_sim::CameraProperties props;
  props.width = 640;
  props.height = 480;
  props.fx = 550.0f;
  props.fy = 550.0f;
  props.cx = props.width / 2;
  props.cy = props.height / 2;
  props.z_near = 0.25f;
  props.z_far = 10.0f;

  // Create the simulation
  gl_depth_sim::SimDepthCamera sim (props);
  sim.add("mesh_identifier", *mesh_ptr, Eigen::Isometry3d::Identity());

  // Spawn a thread to listen for the user to interact with the scene
  std::atomic<bool> thread_done (false);
  std::atomic<bool> save_data (false);

  std::thread input_thread ([&thread_done, &save_data] () {
    char c;
    while (std::cin.get(c))
    {
      if (c == 'q') break;
      if (c == 's') save_data = true;
    }
    thread_done = true;
  });


  // Speak to user
  std::cout << "Press 's' in terminal to save PCD and OpenCV Mat.\n";
  std::cout << "Press 'q' in terminal to quit.\n";

  // State for FPS monitoring
  long frame_counter = 0;

  // In the main (rendering) thread, begin orbiting...
  const auto start = std::chrono::steady_clock::now();
  while (!thread_done)
  {
    double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - start).count();

    const static double radius = 0.75;
    Eigen::Vector3d camera_pos (radius * cos(dt),
                                radius * sin(dt),
                                1.0 + sin(dt));

    Eigen::Vector3d look_at (0,0,0);

    const auto pose = lookat(camera_pos, look_at, Eigen::Vector3d(0,0,1));

    const auto depth_img = sim.render(pose);

    frame_counter++;

    if (frame_counter % 100 == 0)
    {
      std::cout << "FPS: " << frame_counter / dt << "\n";
    }

    if (save_data)
    {
      std::cout << "Saving data...\n";
      save_data = false;
      cv::Mat img;
      gl_depth_sim::toCvImage16u(depth_img, img);
      cv::imwrite("img.png", img);

      pcl::PointCloud<pcl::PointXYZ> cloud;
      gl_depth_sim::toPointCloudXYZ(props, depth_img, cloud);
      pcl::io::savePCDFileBinary("cloud.pcd", cloud);
    }
  }

  input_thread.join();

  return 0;
}
