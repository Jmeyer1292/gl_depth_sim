#ifndef GL_DEPTH_SIM_OPENCV_INTERFACE_H
#define GL_DEPTH_SIM_OPENCV_INTERFACE_H

#include "gl_depth_sim/camera_properties.h"
#include <opencv2/core/core.hpp>

namespace gl_depth_sim
{

/**
 * @brief Converts "native" depth image type to an opencv image with depth data encoded as 32
 * bit floating-point numbers representing the distance of a sample in meters (m) from the image
 * plane.
 */
void toCvImageFp(const DepthImage& depth, cv::Mat& out);

/**
 * @brief Converts "native" depth image type to an opencv image with depth data encoded as 16
 * bit unsigned ints representing the distance of a sample in millimeters (mm) from the image
 * plane. Invalid entries are marked as being at a depth of 0.
 */
void toCvImage16u(const DepthImage& depth, cv::Mat& out);

}

#endif
