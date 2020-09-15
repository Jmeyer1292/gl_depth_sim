#ifndef GL_DEPTH_SIM_SHADER_PROGRAM_H
#define GL_DEPTH_SIM_SHADER_PROGRAM_H

#include <string>
#include <Eigen/Dense>

namespace gl_depth_sim
{

class ShaderProgram
{
public:
  ShaderProgram(const std::string& vertex_shader, const std::string& frag_shader);
  ~ShaderProgram();

  unsigned int id() const { return id_; }

  // Interaction with attributes
  void setInt(const std::string& attr, int val);
  void setUniformMat4(const std::string &attr, const Eigen::Matrix4f& mat);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  unsigned int id_;
};

}

#endif
