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
 #include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/BoneTransformUniformBlock.hpp>
#include <gua/utils/Timer.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>
#include <gua/renderer/SkeletalAnimationUtils.hpp>
#include <vector>
#include <map>
#include <assimp/scene.h>       // Output data structure

namespace gua {

class SkeletalAnimationDirector {
 public:

  SkeletalAnimationDirector(std::shared_ptr<Node> const&);

  inline ~SkeletalAnimationDirector(){};

<<<<<<< HEAD
  void add_animations(aiScene const& scene);
  void add_animations(FbxScene& scene);

<<<<<<< HEAD
  void add_hierarchy(aiScene const& scene);
=======
  void add_animations(aiScene const* scene, std::string const& file_name);
  void add_hierarchy(aiScene const* scene);
>>>>>>> add animation name interface for animation control in avango

=======
>>>>>>> removed unnecessary method from director, gitignore updated
  std::vector<scm::math::mat4f> get_bone_transforms();

  int getBoneID(std::string const& name);
  bool has_anims() const;

  void set_playback_mode(uint mode);
  uint get_playback_mode();
  
  void set_blending_mode(uint mode);
  uint get_blending_mode();

<<<<<<< HEAD
<<<<<<< HEAD
  std::shared_ptr<Node> get_root();
=======
  std::string get_animation()const;
=======
  std::string const& get_animation()const;
>>>>>>> removed unnecessary method from director, gitignore updated
  void        set_animation(std::string);
>>>>>>> add animation name interface for animation control in avango

  float get_blending_factor()const;
  void set_blending_factor(float f);

private:

  void blend_pose(float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::vector<scm::math::mat4f>& transforms);
  void partial_blend(float timeInSeconds, SkeletalAnimation const& pAnim1, SkeletalAnimation const& pAnim2, std::string const& nodeName, std::vector<scm::math::mat4f>& transforms);

  std::map<std::string, int> bone_mapping_; // maps a bone name to its index

  std::shared_ptr<Node> root_;
  std::shared_ptr<Node> anim_start_node_;

  std::vector<std::shared_ptr<SkeletalAnimation>> animations_;
  std::shared_ptr<SkeletalAnimation> currAnimation_;

  enum  Playback {partial = 0, crossfade = 1};
  Playback state_;

  enum  Blending {swap = 0, linear = 1, smoothstep = 2, cosinus = 3};
  Blending blending_state_;
  float blendFactor_;

  unsigned animNum_;
  unsigned animNumLast_;
  uint num_bones_;

  bool firstRun_;
  bool has_anims_;

  float next_blending_end_;
  float blendDuration_ = 0.5f;
  Timer timer_;
};

}

#endif  // GUA_SKELETAL_ANIMATION_DIRECTOR_HPP
