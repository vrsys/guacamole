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

#ifndef GUA_BONE_ANIMATION_HPP
#define GUA_BONE_ANIMATION_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Transformation.hpp>
#include <gua/utils/Mesh.hpp>

namespace fbxsdk_2015_1{
  class FbxTakeInfo;
  class FbxNode;
}

namespace gua {

template<class T>
struct Keyframe {

  Keyframe(double time, T const& value):
    time{time},
    value{value}
  {}

  ~Keyframe(){};

  double time;
  T value;
};

class BoneAnimation {
 public:
  BoneAnimation();

  ~BoneAnimation();

  BoneAnimation(aiNodeAnim* anim);
  BoneAnimation(fbxsdk_2015_1::FbxTakeInfo const& take, fbxsdk_2015_1::FbxNode& node);

  Transformation calculate_transform(float time) const;

  std::string const& get_name() const;

 private:

  scm::math::vec3f interpolate(scm::math::vec3f val1, scm::math::vec3f val2, float factor) const;

  scm::math::quatf interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const;

  template<class T> 
  uint find_key(float animationTime, std::vector<Keyframe<T>> keys) const;

  template<class T> 
  T calculate_value(float time, std::vector<Keyframe<T>> keys) const;

  std::string name;

  std::vector<Keyframe<scm::math::vec3f>> scalingKeys;
  std::vector<Keyframe<scm::math::quatf>> rotationKeys;
  std::vector<Keyframe<scm::math::vec3f>> translationKeys;
};

}

#endif //GUA_BONE_ANIMATION_HPP
