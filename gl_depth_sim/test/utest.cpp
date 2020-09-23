#include <gl_depth_sim/mesh_loader.h>
#include <gl_depth_sim/sim_laser_scanner.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics.hpp>
#include <gtest/gtest.h>
#include <pcl/common/common.h>
#include <ros/package.h>

using namespace gl_depth_sim;

TEST(SimLaserScanner, InsideCylinderTest)
{
  // Create the laser scanner
  LaserScannerProperties properties;
  gl_depth_sim::SimLaserScanner laser_scanner(properties);

  // Load the test mesh
  std::string pkg_path = ros::package::getPath("gl_depth_sim");
  std::unique_ptr<Mesh> mesh_ptr = loadMesh(pkg_path + "/test/cylinder_r1.stl");
  ASSERT_NE(mesh_ptr, nullptr);

  // Add the mesh
  // This mesh is a cylinder with radius = 1.0 and origin in the middle of the cylinder
  laser_scanner.add(*mesh_ptr, Eigen::Isometry3d::Identity());

  // Render the laser scanner at the origin of the cylider with the axis of laser scan "rotation" the same as the cylinder axis
  pcl::PointCloud<pcl::PointXYZ> scan = laser_scanner.render(Eigen::Isometry3d::Identity());

  // Expect the height of the point cloud to be 1
  EXPECT_EQ(scan.height, 1);
  EXPECT_GT(scan.width, 3);

  // Expect this scan to be a circle of radius equal to the cylinder
  const double radius = 1.0;
  namespace ba = boost::accumulators;

  ba::accumulator_set<float, ba::stats<ba::tag::mean, ba::tag::variance>> radius_acc;
  for (const pcl::PointXYZ &pt : scan.points)
  {
    Eigen::Vector3f v(pt.x, pt.y, pt.z);
    radius_acc(v.norm());
  }
  float radius_mean = ba::mean(radius_acc);
  float radius_stdev = std::sqrt(ba::variance(radius_acc));
  std::cout << "Radius mean: " << radius_mean << "; Radius Std. Dev.: " << radius_stdev << std::endl;

  EXPECT_NEAR(radius_mean, radius, 0.0001);

  // Expect the shape to be circular (x/y range equal to diameter) and planar (z-range equal to zero)
  Eigen::Vector4f min_pt, max_pt;
  pcl::getMinMax3D(scan, min_pt, max_pt);
  EXPECT_NEAR(max_pt.x() - min_pt.x(), 2.0 * radius, 0.01);
  EXPECT_NEAR(max_pt.y() - min_pt.y(), 2.0 * radius, 0.01);
  EXPECT_NEAR(max_pt.z() - min_pt.z(), 0.0, 0.01);

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
