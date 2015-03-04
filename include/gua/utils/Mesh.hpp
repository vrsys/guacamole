/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_MESH_HPP
#define GUA_MESH_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>
#include <map>
#include <assimp/scene.h>       // Output data structure
#include <fbxsdk.h>
 
namespace gua {

struct Vertex {
  scm::math::vec3f pos;
  scm::math::vec2f tex;
  scm::math::vec3f normal;
  scm::math::vec3f tangent;
  scm::math::vec3f bitangent;
};

struct Mesh {
 public:
  Mesh();

  Mesh(aiMesh const& mesh);
  Mesh(FbxMesh& mesh);

  void copy_to_buffer(Vertex* vertex_buffer)  const;

  // std::vector<Vertex> vertices;
  std::vector<scm::math::vec3f> positions;
  std::vector<scm::math::vec3f> normals;
  std::vector<scm::math::vec2f> texCoords;
  std::vector<scm::math::vec3f> tangents;
  std::vector<scm::math::vec3f> bitangents;
  std::vector<unsigned> indices;

  unsigned int num_vertices;
  unsigned int num_triangles;

 private:
};


}

#endif //GUA_MESH_HPP
