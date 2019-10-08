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

#ifndef GUA_POSE_HPP
#define GUA_POSE_HPP

#include <gua/skelanim/platform.hpp>

// external headers
#include <map>
#include <memory>

namespace gua
{
class Skeleton;
struct BonePose;

/**
 * @brief holds transformations for bones at one point in an anim
 * @details used to accumulate and blend boneposes
 */
class GUA_SKELANIM_DLL SkeletalPose
{
  public:
    SkeletalPose();

    /**
     * @brief returns if pose contains bonepose
     * @details checks if the pose
     * contains transform for given bone
     *
     * @param name name of bone
     * @return whether pose exists
     */
    bool contains(std::string const& name) const;

    /**
     * @brief returns tranform for given bone
     * @details returns
     *
     * @param name name of bone
     * @return pose of the bone
     */
    BonePose const& get_transform(std::string const& name) const;

    /**
     * @brief sets pose for bone
     * @details sets the transformation
     * of the given bone to given pose
     * adds this bone to the skepose if
     * it wasnt included before
     *
     * @param name name of bone
     * @param value pose to set
     */
    void set_transform(std::string const& name, BonePose const& value);

    /**
     * @brief blends with another pose
     * @details if seconds pose contains poses for
     * bones that are no included in this they
     * are simply added
     *
     * @param pose2 pose to blend with
     * @param blendFactor factor to blend
     */
    void blend(SkeletalPose const& pose2, float blendFactor);

    SkeletalPose& operator+=(SkeletalPose const& pose2);
    SkeletalPose operator+(SkeletalPose const& p2) const;

    SkeletalPose& operator*=(float const factor);
    SkeletalPose operator*(float const factor) const;

    /**
     * @brief replaces part of bone poses
     * @details searches for giben bone and
     * replaces boneposes from there on
     *
     * @param pose2 pose to replace with
     * @param pNode bone from which to start
     * replacing
     */
    void partial_replace(SkeletalPose const& pose2, Skeleton const& skeleton, unsigned bone);

  private:
    std::map<std::string, BonePose> transforms;

    const static BonePose blank_pose;
};

} // namespace gua

#endif // GUA_POSE_HPP
