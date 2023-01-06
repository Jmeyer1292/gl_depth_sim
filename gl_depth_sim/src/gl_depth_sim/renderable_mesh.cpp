#include "gl_depth_sim/renderable_mesh.h"
#include "gl_depth_sim/glad/gl.h"

gl_depth_sim::RenderableMesh::RenderableMesh(const gl_depth_sim::Mesh& mesh)
  : num_indices_{mesh.numIndices()}
{
  // TODO: Do I need to keep around the mesh in this object?
  setupGL(mesh);
}

gl_depth_sim::RenderableMesh::~RenderableMesh()
{
  glDeleteVertexArrays(1, &vao_);
  glDeleteBuffers(1, &vbo_);
  glDeleteBuffers(1, &ebo_);
}

void gl_depth_sim::RenderableMesh::setupGL(const Mesh& mesh)
{
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);

  glBindVertexArray(vao_);

  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof(Eigen::Vector3f) * mesh.vertices().size(), mesh.vertices().data(),
               GL_STATIC_DRAW);

  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) *  mesh.indices().size(), mesh.indices().data(),
               GL_STATIC_DRAW);

  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Eigen::Vector3f), 0);
  glEnableVertexAttribArray(0);

  glBindBuffer(GL_ARRAY_BUFFER, 0);

  glBindVertexArray(0);

}
