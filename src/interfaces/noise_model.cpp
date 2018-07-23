#include "gl_depth_sim/interfaces/noise_model.h"

gl_depth_sim::vector  noise(gl_depth_sim::vector distance)
{
  float segma;
  std::seed_seq seed2{r(), r(), r(), r(), r(), r(), r(), r()};
  std::mt19937 e2(seed2);

  for (int i = 0 ; i < distance.size() ;++i){
  // Seed with a real random value, if available
     //std::random_device r;
     segma= 0.0012+0.0019* pow(distance[i]-0.4,2);
    float r = distance[i];
     // Choose a random mean between 1 and 6
     //std::default_random_engine e1(r());
     //std::uniform_int_distribution<int> uniform_dist(r-segma, r+segma);
     //int mean = uniform_dist(e1);

     // Generate a normal distribution around that mean
     std::normal_distribution<> normal_dist(r, segma);

     float new_value = r + normal_dist(e2);

    }
  return distance;
}
