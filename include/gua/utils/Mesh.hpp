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
#include <gua/config.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>
#include <assimp/scene.h>       // Output data structure

namespace fbxsdk_2015_1{
  class FbxAMatrix;
  class FbxQuaternion;
  class FbxMesh;
}

namespace to_gua{

scm::math::mat4f mat4f(aiMatrix4x4 const& m);
scm::math::quatf quatf(aiQuaternion const& q);

template<typename T>
scm::math::vec3f vec3f(T const& v) {
  scm::math::vec3f res(v[0], v[1], v[2]);
  return res;
}

template<typename T>
scm::math::vec2f vec2f(T const& v) {
  scm::math::vec2f res(v[0], v[1]);
  return res;
}

template<typename T>
scm::math::vec4f vec4f(T const& v) {
  scm::math::vec4 res(v[0], v[1], v[2], v[3]);
  return res;
}

#ifdef GUACAMOLE_FBX
  scm::math::mat4f mat4f(fbxsdk_2015_1::FbxAMatrix const& m);
  scm::math::mat4d mat4d(fbxsdk_2015_1::FbxAMatrix const& m);
  scm::math::quatf quatf(fbxsdk_2015_1::FbxQuaternion const& q);
#endif
}

namespace gua {

struct Mesh {
 public:
  Mesh();

  Mesh(aiMesh const& mesh);
  
#ifdef GUACAMOLE_FBX
  Mesh(fbxsdk_2015_1::FbxMesh& mesh, int material_index = -1);
#endif
  
  struct Vertex {
    scm::math::vec3f pos;
    scm::math::vec2f tex;
    scm::math::vec3f normal;
    scm::math::vec3f tangent;
    scm::math::vec3f bitangent;
  };

  void copy_to_buffer(Vertex* vertex_buffer) const;
  virtual scm::gl::vertex_format get_vertex_format() const;

  std::vector<scm::math::vec3f> positions;
  std::vector<scm::math::vec3f> normals;
  std::vector<scm::math::vec2f> texCoords;
  std::vector<scm::math::vec3f> tangents;
  std::vector<scm::math::vec3f> bitangents;
  std::vector<unsigned> indices;
// TODO: flags, to specify if texcoors are present
  unsigned int num_vertices;
  unsigned int num_triangles;

 protected:

  //struct to save info about future vertex
  struct temp_vert {
    temp_vert(unsigned oindex, unsigned pt, unsigned tr, unsigned ind):
     old_index{oindex},
     point{pt},
     normal{},
     uv{},
     tangent{},
     bitangent{},
     tris{}
    {
      tris.push_back(std::make_pair(tr, ind));
    }
    unsigned old_index;
    unsigned point;
    scm::math::vec3f normal;
    scm::math::vec3f tangent;
    scm::math::vec3f bitangent;
    scm::math::vec2f uv;
    std::vector<std::pair<unsigned, unsigned>> tris; //tris which share vertex
  };
  //struct to save info about future triangle
  struct temp_tri {
    temp_tri(unsigned a, unsigned b, unsigned c):
     verts{a, b, c}
    {}
    std::array<unsigned, 3> verts;
  };

  std::vector<unsigned> construct(fbxsdk_2015_1::FbxMesh& mesh, int material_index);

  template<typename T>
  static std::function<unsigned(temp_vert const&)> get_access_function(FbxLayerElementTemplate<T> const& layer);
};


}

#endif //GUA_MESH_HPP