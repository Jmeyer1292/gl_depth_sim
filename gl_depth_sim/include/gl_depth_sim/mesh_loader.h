#ifndef GL_DEPTH_SIM_MESH_LOADER_H
#define GL_DEPTH_SIM_MESH_LOADER_H

#include "gl_depth_sim/mesh.h"
#include <memory>

namespace gl_depth_sim
{

std::unique_ptr<Mesh> loadMesh(const std::string& path);

}

#endif
