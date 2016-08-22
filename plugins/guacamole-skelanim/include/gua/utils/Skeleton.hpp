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
// #include <gua/Skelanim.hpp>
#if defined (_MSC_VER)
#if defined (GUA_SKELANIM_LIBRARY)
#define GUA_SKELANIM_DLL __declspec( dllexport )
#else
#define GUA_SKELANIM_DLL __declspec( dllimport )
#endif
#else
#define GUA_SKELANIM_DLL
#endif // #if defined(_MSC_VER)
 
#include <gua/utils/Bone.hpp>

// external headers
#include <scm/gl_core.h>
#include <vector>
#include <map>

struct aiScene;
struct aiNode;

namespace gua {

class SkeletalPose;

/**
 * @brief represents one node in skeletal hierarchy
 * @details has methods to traverse skeleton hierarchy
 */
class GUA_SKELANIM_DLL Skeleton {
 public:
  Skeleton() {};
  unsigned addBone(aiNode const& node);
  Skeleton(aiScene const& scene);

#ifdef GUACAMOLE_FBX
  unsigned addBone(FbxNode& node);
  Skeleton(FbxScene& scene);
#endif

  /**
   * @brief collects 
   * @details adds entry for itself to given map
   * and calls method on children
   * 
   * @param ids map to store bone ids
   */
  void collect_indices(std::map<std::string, int>& ids) const;

  /**
   * @brief sets offset matrix and index
   * @details offset matrices may be stored somewhere else
   * therefore they cant be set at construction 
   * 
   * @param infos map with index and offset matrix of each bone
   */
  void set_properties(
      std::map<std::string, std::pair<unsigned int, scm::math::mat4f> > const& infos);


  /**
   * @brief calculates tranform matricex from skeletalpose
   * @details writes own transform matrix at index in vector
   * and calls method on children
   * 
   * @param transformMat4s vector to which matrix is written
   * @param pose pose from which transformation is calculated 
   * @param parentTransform transform matrix of parent bone
   */
  void accumulate_matrices(unsigned start_node,
                           std::vector<scm::math::mat4f>& transformMat4s,
                           SkeletalPose const& pose,
                           scm::math::mat4f const& parentTransform =
                           scm::math::mat4f::identity()) const;

  /**
   * @brief finds bone in hierarchy
   * @details checks if child and given name and
   * calls method on children
   * 
   * @param name name of bone
   * @return pointer to found bone, nullptr if not found
   */
  Bone const* find(std::string const& name) const;

  std::vector<Bone> m_bones;
  std::map<std::string, int> m_mapping;
 private:
  void store_mapping();
};

}

#endif  //GUA_SKELETON_HPP
