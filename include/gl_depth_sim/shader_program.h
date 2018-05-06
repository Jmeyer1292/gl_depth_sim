#ifndef GL_DEPTH_SIM_SHADER_PROGRAM_H
#define GL_DEPTH_SIM_SHADER_PROGRAM_H

#include <glm/glm.hpp>
#include <string>

namespace gl_depth_sim
{

class ShaderProgram
{
public:
  ShaderProgram(const std::string& vertex_shader, const std::string& frag_shader);
  ~ShaderProgram();

  unsigned int id() const { return id_; }

  // Interaction with attributes
  void setUniformMat4(const std::string& attr, const glm::mat4& mat);
  void setInt(const std::string& attr, int val);

private:
  unsigned int id_;
};

}

#endif
