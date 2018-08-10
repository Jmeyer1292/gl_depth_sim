#include "gl_depth_sim/mesh_loader.h"

#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>

#include <iostream>

static std::unique_ptr<gl_depth_sim::Mesh> process(const aiScene* scene)
{
  const int num_meshes = scene->mNumMeshes;
  std::cout << "Scene has " << num_meshes << " meshes\n";


  gl_depth_sim::EigenAlignedVec<Eigen::Vector3f> vertices;
  std::vector<unsigned> indices;

  long index_offset = 0;
  for(int meshNum = 0 ; meshNum <num_meshes ; meshNum++)
  {
    const aiMesh* mesh = scene->mMeshes[meshNum];

    const int nface = mesh->mNumFaces;
    const int nvert = mesh->mNumVertices;
    std::cout << "Mesh has n faces/verts: " << nface << "/" << nvert << "\n";

    for (int i = 0; i < nvert; ++i)
    {
      const aiVector3D& v =  mesh->mVertices[i];
      vertices.push_back({v.x, v.y, v.z});
    }

    int skippedFaces = 0;
    for (int i = 0; i < nface; ++i)
    {
      const aiFace& f = mesh->mFaces[i];

      if(f.mNumIndices == 3)
      {
        indices.push_back(f.mIndices[0] + index_offset);
        indices.push_back(f.mIndices[1] + index_offset);
        indices.push_back(f.mIndices[2] + index_offset);
      }
      else skippedFaces += 1;
    }
    if(skippedFaces > 0)
    {
      std::cout << "Warning: Skipped " << skippedFaces << " malformed faces. \n";
    }
    index_offset += nvert;
  }
  return std::unique_ptr<gl_depth_sim::Mesh>(new gl_depth_sim::Mesh(vertices, indices));
}


std::unique_ptr<gl_depth_sim::Mesh> gl_depth_sim::loadMesh(const std::string& path)
{
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(path, aiProcess_Triangulate);

  if (!scene)
  {
    std::cerr << "Failed to load mesh: " << path << "\n";
    return {};
  }

  return process(scene);

}
