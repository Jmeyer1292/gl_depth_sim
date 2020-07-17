#ifndef SIMLASERSCANNER_H
#define SIMLASERSCANNER_H

#include <gl_depth_sim/sim_depth_camera.h>
#include "gl_depth_sim/camera_properties.h"


class SimLaserScanner
{

  private:
    gl_depth_sim::SimDepthCamera camera_;

  public:
    SimLaserScanner(const gl_depth_sim::SimDepthCamera& camera);

};

SimLaserScanner::SimLaserScanner(const gl_depth_sim::SimDepthCamera& camera)
                 : camera_{camera}
{
  ROS_INFO("INSIDE CONSTRUCTOR");
}



#endif // SIMLASERSCANNER_H
