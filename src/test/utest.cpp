#include <ros/ros.h>
#include <gtest/gtest.h>
#include <boost/foreach.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "gl_depth_sim/interfaces/pcl_interface.h"
#include <pcl_ros/point_cloud.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

PointCloud::ConstPtr cloud_;

void callback(const PointCloud::ConstPtr &msg)
{
  cloud_ = msg;
}


TEST(TestSuite, laser_tests)
{
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("cloud", 1000, &callback);
  ros::topic::waitForMessage<PointCloud>("cloud", ros::Duration(10));
  ASSERT_EQ(5, cloud_->height);
  ASSERT_EQ(2095, cloud_->width);
  for(int i=0;i<cloud_->width; i++){
    for(int j=0; j<cloud_->height; j++){
      pcl::PointXYZ point = cloud_->at(i,j);
      double magnitude = pow(pow(point.x, 2) + pow(point.y, 2) + pow(point.z,2), 0.5);
      EXPECT_TRUE( (magnitude < 1.0005) && (1 > 0.9995) );
    }
  }

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "laser_test");

  ros::AsyncSpinner spinner(1);
  spinner.start();
  int ret = RUN_ALL_TESTS();
  spinner.stop();
  ros::shutdown();
  return ret;
}
