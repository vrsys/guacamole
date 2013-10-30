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

#ifndef GUA_FRUSTUM_HPP
#define GUA_FRUSTUM_HPP

// guacamole headers
#include <gua/math/BoundingBox.hpp>
#include <gua/math/math.hpp>

namespace gua {

/**
 *
 */
class Frustum {

 public:

  Frustum() {}

  Frustum(math::mat4 const& camera_transform,
          math::mat4 const& screen_transform,
          float clip_near,
          float clip_far);

  inline math::vec3 const& get_camera_position() const {
    return camera_position_;
  }
  inline math::mat4 const& get_projection() const { return projection_; }
  inline math::mat4 const& get_view() const { return view_; }

  bool is_inside(math::BoundingBox<math::vec3> const& bbox) const;

 private:

  math::vec3 camera_position_;
  math::mat4 projection_;
  math::mat4 view_;
  std::vector<math::vec4> planes_;

};

}

#endif  // GUA_FRUSTUM_HPP
