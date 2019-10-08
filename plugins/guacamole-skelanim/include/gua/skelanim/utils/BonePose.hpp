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

#ifndef GUA_BONEPOSE_HPP
#define GUA_BONEPOSE_HPP

#include <gua/skelanim/platform.hpp>

// external headers
#include <scm/gl_core.h>
#include <scm/core/math/quat.h>

namespace gua
{
/**
 * @brief holds transformation of bone
 * @details can be blended with another bone pose
 */
struct GUA_SKELANIM_DLL BonePose
{
  public:
    BonePose();

    BonePose(scm::math::vec3f const& scale, scm::math::quatf const& rotate, scm::math::vec3f const& translate);

    ~BonePose();

    scm::math::mat4f to_matrix() const;

    BonePose blend(BonePose const& t, float const factor) const;

    BonePose operator+(BonePose const& t) const;
    BonePose& operator+=(BonePose const& t);

    BonePose operator*(float const factor) const;
    BonePose& operator*=(float const f);

  private:
    scm::math::vec3f scaling;
    scm::math::quatf rotation;
    scm::math::vec3f translation;
};

} // namespace gua

#endif // GUA_BONEPOSE_HPP