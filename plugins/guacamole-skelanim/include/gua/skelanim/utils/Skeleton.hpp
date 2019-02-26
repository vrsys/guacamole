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

#ifndef GUA_SKELETON_HPP
#define GUA_SKELETON_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/utils/fbxfwd.hpp>
#include <gua/skelanim/platform.hpp>
#include <gua/skelanim/utils/Bone.hpp>

// external headers
#include <scm/gl_core.h>
#include <vector>
#include <map>

struct aiScene;
struct aiNode;

namespace gua
{
class SkeletalPose;

/**
 * @brief represents one node in skeletal hierarchy
 * @details has methods to traverse skeleton hierarchy
 */
class GUA_SKELANIM_DLL Skeleton
{
  public:
    inline Skeleton(){};
    Skeleton(aiScene const& scene);
    unsigned addBone(aiNode const& node);

#ifdef GUACAMOLE_FBX
    Skeleton(FbxScene& scene);
    unsigned addBone(FbxNode& node);
#endif

    /**
     * @brief collects bone associations
     * @details collects mapping from bone name to index
     * @return map of associations
     */
    std::map<std::string, int> const& get_mapping() const;

    /**
     * @brief calculates tranform matrices from skeletalpose
     * @details returns transform matrices for all bones of skeleton
     *
     * @param pose pose from which transformation is calculated
     * @return vector for transformation matrices
     */
    std::vector<scm::math::mat4f> accumulate_matrices(unsigned index_bone, SkeletalPose const& pose) const;

    /**
     * @brief finds bone in hierarchy
     * @details
     * @param name name of bone
     * @return index >= 0, -1 if not found
     */
    int find(std::string const& name) const;

    Bone const& get(std::size_t index) const;

    std::size_t num_bones() const;

    std::vector<Bone> const& get_bones() const;
    void set_bones(std::vector<Bone> const&);

    /**
     * @brief calculates tranform matrices from skeletalpose
     * @details writes transform matrices at index in vector
     * and calls method on children of bone
     *
     * @param index_bone index of bone to process
     * @param transformMat4s vector to which matrix is written
     * @param pose pose from which transformation is calculated
     * @param parentTransform transform matrix of parent bone
     */
    void
    accumulate_matrices(unsigned index_bone, std::vector<scm::math::mat4f>& transformMat4s, SkeletalPose const& pose, scm::math::mat4f const& parentTransform = scm::math::mat4f::identity()) const;

  private:
    /**
     * @brief sets offset matrix and index
     * @details offset matrices may be stored somewhere else
     * therefore they cant be set at construction
     *
     * @param offsets map with offset matrix of each mapped bone
     */
    void set_offsets(std::map<std::string, scm::math::mat4f> const& offsets);

    void store_mapping();

    std::vector<Bone> m_bones;
    std::map<std::string, int> m_mapping;
};

} // namespace gua

#endif // GUA_SKELETON_HPP
