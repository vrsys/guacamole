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

#ifndef GUA_SKELETAL_ANIMATION_HPP
#define GUA_SKELETAL_ANIMATION_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/utils/Mesh.hpp>
#include <gua/utils/SkeletalPose.hpp>
#include <gua/utils/BoneAnimation.hpp>

#include <vector>

namespace fbxsdk_2015_1{
  class FbxNode;
  class FbxAnimStack;
}

namespace gua {
class SkeletalPose;

class SkeletalAnimation {
 public:
  SkeletalAnimation();

  SkeletalAnimation(aiAnimation const& anim, std::string const& name = "");
  SkeletalAnimation(fbxsdk_2015_1::FbxAnimStack* anim, std::vector<fbxsdk_2015_1::FbxNode*> const& bones, std::string const& name = "");

  ~SkeletalAnimation();

  SkeletalPose calculate_pose(float time) const;

  double get_duration() const;
  std::string const& get_name() const;

 private:
  std::string name;
  unsigned numFrames;
  double numFPS;
  double duration;
  unsigned numBoneAnims;

  std::vector<BoneAnimation> boneAnims;
};

}

#endif //GUA_SKELETAL_ANIMATION_HPP
