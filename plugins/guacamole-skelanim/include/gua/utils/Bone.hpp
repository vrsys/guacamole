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

#ifndef GUA_BONE_HPP
#define GUA_BONE_HPP

// guacamole headers
#include <gua/config.hpp>
#include <gua/utils/fbxfwd.hpp>
#include <gua/Skelanim.hpp>

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
class GUA_SKELANIM_DLL Bone {
 public:
  Bone();
  Bone(aiNode const& node);
  Bone(aiScene const& scene);

#ifdef GUACAMOLE_FBX
  Bone(FbxNode& node);
  Bone(FbxScene& scene);
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
   * @brief collects 
   * @details adds itself to given vector
   * and calls method on children
   * 
   * @param bones container to store bones
   */
  void collect_bones(std::vector<Bone>& bones) const;
  void create_bones(std::vector<Bone> const& bones);

  /**
   * @brief sets offset matrix and index
   * @details offset matrices may be stored somewhere else
   * therefore they cant be set at construction 
   * 
   * @param infos map with index and offset matrix of each bone
   */
  void set_properties(
      std::map<std::string, scm::math::mat4f> const& infos, std::size_t& idx);


  /**
   * @brief calculates tranform matricex from skeletalpose
   * @details writes own transform matrix at index in vector
   * and calls method on children
   * 
   * @param transformMat4s vector to which matrix is written
   * @param pose pose from which transformation is calculated 
   * @param parentTransform transform matrix of parent bone
   */
  void accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s,
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
  std::shared_ptr<Bone> find(std::string const& name) const;

  std::size_t num_bones() const;

  std::string name;
  std::vector<std::shared_ptr<Bone> > children;

 private:
  std::vector<unsigned> children2;

  int index;
  std::string parentName;
  unsigned numChildren;
  scm::math::mat4f transformation;
  //transforms to bone space
  scm::math::mat4f offsetMatrix;
};

}

#endif  //GUA_BONE_HPP
