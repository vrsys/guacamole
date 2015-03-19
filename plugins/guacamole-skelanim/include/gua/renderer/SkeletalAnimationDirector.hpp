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

#ifndef GUA_SKELETAL_ANIMATION_DIRECTOR_HPP
#define GUA_SKELETAL_ANIMATION_DIRECTOR_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/SkeletalAnimation.hpp>

#include <gua/utils/Bone.hpp>
#include <gua/utils/Timer.hpp>

// external headers
#include <vector>

namespace fbxsdk_2015_1{
  class FbxScene;
}

namespace gua {

// class SkeletalAnimationNode;

class SkeletalAnimationDirector {

  friend class SkeletalAnimationNode;

 public:

  SkeletalAnimationDirector(std::shared_ptr<Bone> const&);

  inline ~SkeletalAnimationDirector(){};

  void add_animations(aiScene const& scene, std::string const& name);
  void add_animations(fbxsdk_2015_1::FbxScene& scene, std::string const& name);

  std::vector<scm::math::mat4f> get_bone_transforms();

// private:

  void calculate_matrices(float TimeInSeconds, SkeletalAnimation const& pAnim, std::vector<scm::math::mat4f>& Transforms);
  void calculate_matrices(std::vector<scm::math::mat4f>& Transforms);

  void blend_pose(float timeInSeconds1, float timeInSeconds2, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::vector<scm::math::mat4f>& transforms);
  void partial_blend(float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms);

  std::map<std::string, int> bone_mapping_; // maps a bone name to its index

  std::shared_ptr<Bone> root_;
  std::shared_ptr<Bone> anim_start_node_;

  std::map<std::string, SkeletalAnimation> animations_;

  float blend_factor_;

  std::string anim_1_;
  std::string anim_2_;
  float anim_time_1_;
  float anim_time_2_;
  uint num_bones_;

  bool firstRun_;
  bool has_anims_;
  // bool loop_;
  // bool loop_prev_;

  // float blend_start_;
  // float blend_start_prev_;
  // float blend_end_;
  // float blend_duration_;
  // Timer timer_;

  const static std::string none_loaded;
};

}

#endif  // GUA_SKELETAL_ANIMATION_DIRECTOR_HPP
