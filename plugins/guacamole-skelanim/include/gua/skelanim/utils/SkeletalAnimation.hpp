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
#include <gua/config.hpp>
#include <gua/utils/fbxfwd.hpp>
#include <gua/skelanim/utils/BoneAnimation.hpp>
#include <gua/skelanim/platform.hpp>

// external headers
#include <vector>
#include <string>

struct aiAnimation;

namespace gua
{
class SkeletalPose;

/**
 * @brief holds bone animations for one animation
 */
class GUA_SKELANIM_DLL SkeletalAnimation
{
  public:
    SkeletalAnimation();

    SkeletalAnimation(aiAnimation const& anim, std::string const& name = "");
#ifdef GUACAMOLE_FBX
    SkeletalAnimation(FbxAnimStack* anim, std::vector<FbxNode*> const& bones, std::string const& name = "");
#endif
    ~SkeletalAnimation();

    /**
     * @brief calculates skelpose from this anim
     * @details calculates the pose at the given time
     *
     * @param time time for which to calculate pose
     * @return SkeletalPose at given time
     */
    SkeletalPose calculate_pose(float time) const;

    /**
     * @brief returns anim duration
     * @return duration in seconds
     */
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

} // namespace gua

#endif // GUA_SKELETAL_ANIMATION_HPP
