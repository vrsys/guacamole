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
#include <gua/config.hpp>
#include <gua/utils/fbxfwd.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

struct aiNodeAnim;

namespace gua
{
struct BonePose;

/**
 * @brief holds transformation at one point in time
 */
template <class T>
struct Keyframe
{
    Keyframe(double time, T const& value) : time{time}, value{value} {}

    ~Keyframe(){};

    double time;
    T value;
};

/**
 * @brief keyframe container
 * @details holds transformations for one bone throughout one animation
 */
class BoneAnimation
{
  public:
    BoneAnimation();

    BoneAnimation(aiNodeAnim* anim);

#ifdef GUACAMOLE_FBX
    BoneAnimation(FbxTakeInfo const& take, FbxNode& node);
#endif
    ~BoneAnimation();

    /**
     * @brief return pose at given time
     *
     * @param time normalized time
     * @return interpolated pose
     */
    BonePose calculate_pose(float time) const;

    std::string const& get_name() const;

  private:
    scm::math::vec3f interpolate(scm::math::vec3f val1, scm::math::vec3f val2, float factor) const;
    scm::math::quatf interpolate(scm::math::quatf val1, scm::math::quatf val2, float factor) const;

    /**
     * @brief finds keyframe closest to given time
     * @details finds last keyframe before given time
     *
     * @param animationTime normalized time
     * @param keys vector of keyframes to search in
     * @return index of keyframe
     */
    template <class T>
    int find_key(float animationTime, std::vector<Keyframe<T>> keys) const;

    /**
     * @brief returns transformation at given time
     * @details searches for two closest keyframes
     *  and interpolates them
     *
     * @param time normalized time
     * @param keys vector of keyframes ot use
     *
     * @return interpolated transformation
     */
    template <class T>
    T calculate_value(float time, std::vector<Keyframe<T>> keys) const;

    std::string name;
    std::vector<Keyframe<scm::math::vec3f>> scalingKeys;
    std::vector<Keyframe<scm::math::quatf>> rotationKeys;
    std::vector<Keyframe<scm::math::vec3f>> translationKeys;
};

} // namespace gua

#endif // GUA_BONE_ANIMATION_HPP
