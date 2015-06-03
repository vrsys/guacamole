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

#ifndef GUA_SKELETALTRANSFORMATION_HPP
#define GUA_SKELETALTRANSFORMATION_HPP

// external headers
#include <scm/gl_core.h>
#include <vector>
#include <memory>

namespace gua {
  class Bone;
  class SkeletalAnimation;

namespace SkeletalTransformation {

void from_hierarchy(std::shared_ptr<Bone> const& anim_start_node_,
                    std::vector<scm::math::mat4f>& Transforms);

void from_anim(std::shared_ptr<Bone> const& anim_start_node_,
               float time_normalized,
               SkeletalAnimation const& anim,
               std::vector<scm::math::mat4f>& Transforms);

void blend_anims(std::shared_ptr<Bone> const& anim_start_node_,
                 float blend_factor,
                 float time_normalized1,
                 float time_normalized2,
                 SkeletalAnimation const& anim_1,
                 SkeletalAnimation const& anim_2,
                 std::vector<scm::math::mat4f>& transforms);

void partial_blend(std::shared_ptr<Bone> const& anim_start_node_,
                   float time_normalized1,
                   float time_normalized2,
                   SkeletalAnimation const& anim_1,
                   SkeletalAnimation const& anim_2,
                   std::string const& split_node_name,
                   std::vector<scm::math::mat4f>& transforms);
}
}

#endif  // GUA_SKELETALTRANSFORMATION_HPP
