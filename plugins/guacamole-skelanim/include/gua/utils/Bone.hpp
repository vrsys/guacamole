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
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/SkeletalPose.hpp>
#include <gua/utils/SkeletalAnimation.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>
#include <vector>
#include <map>

class aiScene;

namespace fbxsdk_2015_1{
  class FbxNode;
  class FbxScene;
}

namespace gua {
  
class Bone {
 public:
  Bone();
  Bone(aiNode const& node);
  Bone(fbxsdk_2015_1::FbxNode& node);
  Bone(aiScene const& scene);
  Bone(fbxsdk_2015_1::FbxScene& scene);

  ~Bone();

  void collect_indices(std::map<std::string, int>& ids) const;

  void set_properties(std::map<std::string, std::pair<uint, scm::math::mat4f>> const& infos);

  std::shared_ptr<Bone> find(std::string const& name) const;


  void calculate_matrices(std::vector<scm::math::mat4f>& Transforms) const;
  void calculate_matrices(float TimeInSeconds, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& Transforms) const;

  void blend_pose(float blend_factor, float timeInSeconds1, float timeInSeconds2, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::vector<scm::math::mat4f>& transforms) const;
  // void partial_blend(float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms) const;

  std::string name;
  std::vector<std::shared_ptr<Bone>> children;
  
 private:
  void accumulate_matrices(std::vector<scm::math::mat4f>& transformMat4s, SkeletalPose const& pose, scm::math::mat4f const& parentTransform) const;

  int index;
  std::string parentName;
  unsigned numChildren;
  scm::math::mat4f transformation;
  //transforms to bone space
  scm::math::mat4f offsetMatrix;
};

}

#endif //GUA_BONE_HPP