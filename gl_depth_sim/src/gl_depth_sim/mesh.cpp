#include "gl_depth_sim/mesh.h"

gl_depth_sim::Mesh::Mesh(const EigenAlignedVec<Eigen::Vector3f>& vertices, const std::vector<unsigned>& indices)
  : vertices_(vertices), indices_(indices)
{}
