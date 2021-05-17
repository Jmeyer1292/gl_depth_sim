#include "gl_depth_sim/interfaces/opencv_interface.h"

void gl_depth_sim::toCvImageFp(const gl_depth_sim::DepthImage& depth, cv::Mat& out)
{
  out.create(depth.rows, depth.cols, CV_32FC1);
  for (int y = 0; y < depth.rows; ++y)
  {
    for (int x = 0; x < depth.cols; ++x)
    {
      out.at<float>(y, x) = depth.distance(y, x);
    }
  }
}

void gl_depth_sim::toCvImage16u(const gl_depth_sim::DepthImage& depth, cv::Mat& out)
{
  out.create(depth.rows, depth.cols, CV_16UC1);
  for (int y = 0; y < depth.rows; ++y)
  {
    for (int x = 0; x < depth.cols; ++x)
    {
      out.at<unsigned short>(y, x) = 1000.0f * depth.distance(y, x);
    }
  }
}
