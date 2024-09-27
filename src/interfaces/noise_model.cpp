#include "gl_depth_sim/interfaces/noise_model.h"

std::vector<float> gl_depth_sim::noise(std::vector<float> distance)
{
  float segma;
  std::random_device r;
  std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()};
  std::mt19937 e2(seed2);

  for (int i = 0 ; i < distance.size() ;++i){
  // Seed with a real random value, if available
    segma= 0.0012+0.0019* pow(distance[i]-0.4,2);
    float r = distance[i];
    // Generate a normal distribution around that mean
    std::normal_distribution<> normal_dist(r, segma);
    distance[i] = normal_dist(e2);
    }
  return distance;
}
