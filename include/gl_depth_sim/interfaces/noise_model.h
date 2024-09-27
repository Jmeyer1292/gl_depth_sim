#ifndef NOISE_MODEL_H
#define NOISE_MODEL_H

#include <math.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>

namespace gl_depth_sim
{
std::vector<float> noise(std::vector<float> distance);

}
#endif // NOISE_MODEL_H
