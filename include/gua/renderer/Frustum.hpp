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

  Frustum();

  static Frustum perspective(math::mat4 const& camera_transform,
                             math::mat4 const& screen_transform,
                             float clip_near,
                             float clip_far);

  static Frustum orthographic(math::mat4 const& camera_transform,
                              math::mat4 const& screen_transform,
                              float clip_near,
                              float clip_far);

  std::vector<math::vec3> get_corners() const;

  inline math::vec3 get_camera_position() const {
    return math::vec3(camera_transform_.column(3)[0],
                      camera_transform_.column(3)[1],
                      camera_transform_.column(3)[2]);
  }

  inline math::mat4 const& get_camera_transform() const { return camera_transform_; }
  inline math::mat4 const& get_screen_transform() const { return screen_transform_; }

  inline math::mat4 const& get_projection() const { return projection_; }
  inline math::mat4 const& get_view() const { return view_; }
  inline float get_clip_near() const { return clip_near_; }
  inline float get_clip_far() const { return clip_far_; }

  bool is_inside(math::BoundingBox<math::vec3> const& bbox) const;

 private:

  static void init_frustum_members(math::mat4 const& camera_transform,
                         math::mat4 const& screen_transform,
                         Frustum& frustum);

  math::mat4 camera_transform_;
  math::mat4 screen_transform_;
  math::mat4 projection_;
  math::mat4 view_;
  std::vector<math::vec4> planes_;
  float clip_near_;
  float clip_far_;

};

}

#endif  // GUA_FRUSTUM_HPP
