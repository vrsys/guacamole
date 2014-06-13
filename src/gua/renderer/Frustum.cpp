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

// class header
#include <gua/renderer/Frustum.hpp>

#include <iostream>

namespace gua {

Frustum::Frustum()
    : camera_transform_(math::mat4::identity()),
      screen_transform_(math::mat4::identity()),
      projection_(math::mat4::identity()),
      view_(math::mat4::identity()),
      planes_(6),
      clip_near_(0),
      clip_far_(0) {}

////////////////////////////////////////////////////////////////////////////////

Frustum Frustum::perspective(math::mat4 const& camera_transform,
                    math::mat4 const& screen_transform,
                    float clip_near,
                    float clip_far) {

  auto projection = math::compute_perspective_frustum(
             camera_transform.column(3), screen_transform, clip_near, clip_far);

  Frustum result;

  result.projection_ = projection;
  result.clip_near_ = clip_near;
  result.clip_far_ = clip_far;

  init_frustum_members(camera_transform, screen_transform, result);

  return result;
}

////////////////////////////////////////////////////////////////////////////////

Frustum Frustum::orthographic(math::mat4 const& camera_transform,
                    math::mat4 const& screen_transform,
                    float clip_near,
                    float clip_far) {

  auto projection = math::compute_orthographic_frustum(
             camera_transform.column(3), screen_transform, clip_near, clip_far);

  Frustum result;

  result.projection_ = projection;
  result.clip_near_ = clip_near;
  result.clip_far_ = clip_far;

  init_frustum_members(camera_transform, screen_transform, result);

  return result;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<math::vec3> Frustum::get_corners() const {
  std::vector<math::vec4> tmp(8);
  std::vector<math::vec3> result(8);

  auto inverse_transform(scm::math::inverse(projection_ * view_));

  tmp[0] = inverse_transform * math::vec4(-1, -1, -1, 1);
  tmp[1] = inverse_transform * math::vec4(-1, -1,  1, 1);
  tmp[2] = inverse_transform * math::vec4(-1,  1, -1, 1);
  tmp[3] = inverse_transform * math::vec4(-1,  1,  1, 1);
  tmp[4] = inverse_transform * math::vec4( 1, -1, -1, 1);
  tmp[5] = inverse_transform * math::vec4( 1, -1,  1, 1);
  tmp[6] = inverse_transform * math::vec4( 1,  1, -1, 1);
  tmp[7] = inverse_transform * math::vec4( 1,  1,  1, 1);

  for (int i(0); i<8; ++i) {
    result[i] = tmp[i]/tmp[i][3];
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////////

bool Frustum::is_inside(math::BoundingBox<math::vec3> const& bbox) const {

  auto distance = [](math::vec4 const & plane, math::vec3 const & point) {
    return plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] +
           plane[3];
  }
  ;

  for (unsigned i(0); i < 6; ++i) {

    auto p(bbox.min);
    if (planes_[i][0] >= 0)
      p[0] = bbox.max[0];
    if (planes_[i][1] >= 0)
      p[1] = bbox.max[1];
    if (planes_[i][2] >= 0)
      p[2] = bbox.max[2];

    // is the positive vertex outside?
    if (distance(planes_[i], p) < 0)
      return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

void Frustum::init_frustum_members(math::mat4 const& camera_transform,
                         math::mat4 const& screen_transform,
                         Frustum& frustum) {

  math::mat4 view_transform(screen_transform);
  view_transform[12] = 0.f;
  view_transform[13] = 0.f;
  view_transform[14] = 0.f;
  view_transform[15] = 1.f;

  frustum.camera_transform_ = camera_transform;
  frustum.screen_transform_ = screen_transform;

  view_transform =
      scm::math::make_translation(frustum.get_camera_position()) * view_transform;

  frustum.view_ = scm::math::inverse(view_transform);

  auto projection_view(frustum.projection_ * frustum.view_);

  //store normals

  //left plane
  frustum.planes_[0] = math::vec4(projection_view[3] + projection_view[0],
                          projection_view[7] + projection_view[4],
                          projection_view[11] + projection_view[8],
                          projection_view[15] + projection_view[12]);

  //right plane
  frustum.planes_[1] = math::vec4(projection_view[3] - projection_view[0],
                          projection_view[7] - projection_view[4],
                          projection_view[11] - projection_view[8],
                          projection_view[15] - projection_view[12]);

  //bottom plane
  frustum.planes_[2] = math::vec4(projection_view[3] + projection_view[1],
                          projection_view[7] + projection_view[5],
                          projection_view[11] + projection_view[9],
                          projection_view[15] + projection_view[13]);

  //top plane
  frustum.planes_[3] = math::vec4(projection_view[3] - projection_view[1],
                          projection_view[7] - projection_view[5],
                          projection_view[11] - projection_view[9],
                          projection_view[15] - projection_view[13]);

  //near plane
  frustum.planes_[4] = math::vec4(projection_view[3] + projection_view[2],
                          projection_view[7] + projection_view[6],
                          projection_view[11] + projection_view[10],
                          projection_view[15] + projection_view[14]);

  //far plane
  frustum.planes_[5] = math::vec4(projection_view[3] - projection_view[2],
                          projection_view[7] - projection_view[6],
                          projection_view[11] - projection_view[10],
                          projection_view[15] - projection_view[14]);
}


}
