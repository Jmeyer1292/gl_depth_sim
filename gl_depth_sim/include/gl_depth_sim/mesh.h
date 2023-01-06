#ifndef GL_DEPTH_SIM_MESH_H
#define GL_DEPTH_SIM_MESH_H

#include <Eigen/Dense>
#include <vector>

namespace gl_depth_sim
{

template <typename T>
using EigenAlignedVec = std::vector<T, Eigen::aligned_allocator<T>>;

class Mesh
{
public:
  Mesh(const EigenAlignedVec<Eigen::Vector3f>& vertices, const std::vector<unsigned>& indices);

  std::size_t numIndices() const { return indices_.size(); }
  std::size_t numVertices() const { return vertices_.size(); }

  const std::vector<unsigned>& indices() const { return indices_; }
  const EigenAlignedVec<Eigen::Vector3f>& vertices() const { return vertices_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  EigenAlignedVec<Eigen::Vector3f> vertices_;
  std::vector<unsigned> indices_;
};

}

#endif // GL_DEPTH_SIM_MESH_H
