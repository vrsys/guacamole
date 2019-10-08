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
#include <gua/skelanim/platform.hpp>
#include <gua/skelanim/utils/Skeleton.hpp>
#include <gua/utils/Mesh.hpp>
#include <gua/utils/fbxfwd.hpp>
#include <gua/config.hpp>
// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>
#include <vector>

struct aiMesh;

namespace gua
{
/**
 * @brief mesh with vertex-bone mapping
 */
struct GUA_SKELANIM_DLL SkinnedMesh : public Mesh
{
  public:
    SkinnedMesh();

    SkinnedMesh(aiMesh const& mesh, Skeleton const& skeleton = Skeleton{});

#ifdef GUACAMOLE_FBX
    SkinnedMesh(FbxMesh& mesh, Skeleton const& skeleton = Skeleton{}, unsigned const material_index = 0);
#endif
    /**
     * @brief holds information of a skinned vertex
     */
    struct Vertex
    {
        scm::math::vec3f pos;
        scm::math::vec2f tex;
        scm::math::vec3f normal;
        scm::math::vec3f tangent;
        scm::math::vec3f bitangent;
        unsigned int bone_id_offset;
        unsigned int nr_of_bones;
    };

    /**
     * @brief writes vertex info to given buffer
     * @details writes vertex info to buffer
     * and offsets weight info
     *
     * @param vertex_buffer buffer to write to
     * @param resource_offset weight info offset
     */
    void copy_to_buffer(Vertex* vertex_buffer, unsigned int resource_offset) const;

    /**
     * @brief returns vertex layout for skinned mesh vertex
     * @return schism vertex format
     */
    scm::gl::vertex_format get_vertex_format() const override;

    std::vector<unsigned int> bone_ids;
    std::vector<float> bone_weights;
    std::vector<unsigned int> bone_counts;

    std::vector<unsigned int> const& get_bone_ids() const;
    std::vector<float> const& get_bone_weights() const;

  private:
    /**
     * @brief holds information for influences on one vertex
     */
    struct bone_influences
    {
        std::vector<unsigned int> IDs;
        std::vector<float> weights;

        bone_influences() : IDs{}, weights{} {};

        void add_bone(unsigned int bone_ID, float weight)
        {
            IDs.push_back(bone_ID);
            weights.push_back(weight);
        }
    };

    /**
     * @brief create bone influences from assimp mesh and hierarchy
     * @details collects the bone indices from the hierarchy and
     * mapping info from the mesh
     *
     * @param mesh skinned assimp mesh
     * @param skeleton the bone hierarchy
     * @return the calculated bone influences
     */
    static std::vector<bone_influences> get_weights(aiMesh const& mesh, Skeleton const& skeleton);

    /**
     * @brief create bone influences from fbx mesh and hierarchy
     * @details collects the bone indices from the hierarchy and
     * mapping info from the mesh
     *
     * @param mesh skinned fbx mesh
     * @param skeleton the bone hierarchy
     *
     * @return the calculated bone influences
     */
#ifdef GUACAMOLE_FBX
    static std::vector<bone_influences> get_weights(FbxMesh const& mesh, Skeleton const& skeleton);
#endif
};

} // namespace gua

#endif // GUA_SKINNED_MESH_HPP
