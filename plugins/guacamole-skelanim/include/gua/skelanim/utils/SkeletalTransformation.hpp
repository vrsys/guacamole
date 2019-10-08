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

#include <gua/skelanim/platform.hpp>

// external headers
#include <scm/gl_core.h>
#include <vector>
#include <memory>

namespace gua
{
class Skeleton;
class SkeletalAnimation;

namespace SkeletalTransformation
{
/**
 * @brief creates transformations from hierarchy
 * @details returns the binding/default transformation
 *  stored in the skeleton
 *
 * @param skeleton the bone hierarchy to use
 * @param anim_start_node node from which to start calculating transforms
 * @param Transforms vector to write transforms to
 */
GUA_SKELANIM_DLL std::vector<scm::math::mat4f> from_hierarchy(Skeleton const& skeleton, unsigned anim_start_node);
/**
 * @brief creates transformations from anim at given time
 * @details calculates transforms from the pose at given time
 *
 * @param skeleton the bone hierarchy to use
 * @param anim_start_node node from which to start calculating transforms
 * @param time_normalized time for which to calculate transforms
 * @param anim anim from which to calculate pose
 * @param Transforms vector to write transforms to
 */
GUA_SKELANIM_DLL std::vector<scm::math::mat4f> from_anim(Skeleton const& skeleton, unsigned anim_start_node, float time_normalized, SkeletalAnimation const& anim);
/**
 * @brief calculates two poses and blends them
 * @details calculates poses for both anims at given time and blend swith blend factor
 *
 * @param skeleton the bone hierarchy to use
 * @param anim_start_node node from which to start calculating transforms
 * @param blend_factor factor to blend anims with
 * @param time_normalized1 time for which to calculate transforms from anim_1
 * @param time_normalized2 time for which to calculate transforms frim anim_2
 * @param anim_1 first anim
 * @param anim_2 second anim
 * @param transforms vector to write transforms to
 // */
GUA_SKELANIM_DLL std::vector<scm::math::mat4f>
blend_anims(Skeleton const& skeleton, unsigned anim_start_node, float blend_factor, float time_normalized1, float time_normalized2, SkeletalAnimation const& anim_1, SkeletalAnimation const& anim_2);
/**
 * @brief calculates two poses and changes them from node on
 * @details overwrites first pose with second pose on from given node
 *
 * @param skeleton the bone hierarchy to use
 * @param anim_start_node node from which to start calculating transforms
 * @param time_normalized1 time for which to calculate transforms from anim_1
 * @param time_normalized2 time for which to calculate transforms frim anim_2
 * @param anim_1 base anim
 * @param anim_2 anim to override base anim with
 * @param node_name node form which to start overwriting pose1 with pose2
 * @param transforms vector to write transforms to
 */
GUA_SKELANIM_DLL std::vector<scm::math::mat4f> partial_blend(Skeleton const& skeleton,
                                                             unsigned anim_start_node,
                                                             float time_normalized1,
                                                             float time_normalized2,
                                                             SkeletalAnimation const& anim_1,
                                                             SkeletalAnimation const& anim_2,
                                                             std::string const& split_node_name);
} // namespace SkeletalTransformation
} // namespace gua

#endif // GUA_SKELETALTRANSFORMATION_HPP
