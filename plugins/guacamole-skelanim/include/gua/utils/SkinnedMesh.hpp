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

#ifndef GUA_SKINNED_MESH_HPP
#define GUA_SKINNED_MESH_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Bone.hpp>
#include <gua/utils/Mesh.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

#include <vector>
#include <map>
#include <assimp/scene.h>
#include <fbxsdk.h>
 
namespace gua {

#define ZERO_MEM(a) memset(a, 0, sizeof(a))
struct weight_map
{        
  uint IDs[4];
  float weights[4];

  weight_map()
  {
      Reset();
  };
  
  void Reset()
  {
      ZERO_MEM(IDs);
      ZERO_MEM(weights);        
  }
  
  void AddBoneData(uint bone_ID, float weight)
  {
    uint num_weights = (sizeof(IDs)/sizeof(IDs[0]));
    for (uint i = 0 ; i <  num_weights; i++) {
        if (weights[i] == 0.0) {
            IDs[i]     = bone_ID;
            weights[i] = weight;
            return;
        }        
    }
    // should never get here - more bones than we have space for
    Logger::LOG_WARNING << "Warning: Ignoring bone associated to vertex (more than " << num_weights << ")" << std::endl;
    //assert(false);
  }
};

struct SkinnedVertex {
  scm::math::vec3f pos;
  scm::math::vec2f tex;
  scm::math::vec3f normal;
  scm::math::vec3f tangent;
  scm::math::vec3f bitangent;
  scm::math::vec4f bone_weights;
  scm::math::vec4i bone_ids;
};

struct SkinnedMesh {
 public:
  SkinnedMesh();

  SkinnedMesh(aiMesh const& mesh, Bone const& root = Bone{});
  SkinnedMesh(FbxMesh& mesh, Bone const& root = Bone{});

  void copy_to_buffer(SkinnedVertex* vertex_buffer)  const;
  scm::gl::vertex_format get_vertex_format()  const;

  // std::vector<Vertex> vertices;
  std::vector<scm::math::vec3f> positions;
  std::vector<scm::math::vec3f> normals;
  std::vector<scm::math::vec2f> texCoords;
  std::vector<scm::math::vec3f> tangents;
  std::vector<scm::math::vec3f> bitangents;
  std::vector<weight_map> weights;
  std::vector<unsigned> indices;

  unsigned int num_vertices;
  unsigned int num_triangles;

 private:
  void init_weights(aiMesh const& mesh, Bone const& root);
  std::vector<weight_map> get_weights(FbxMesh const& mesh, Bone const& root);
};

}

#endif //GUA_SKINNED_MESH_HPP
