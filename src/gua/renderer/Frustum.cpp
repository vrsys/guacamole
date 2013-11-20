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

Frustum::Frustum(math::mat4 const& camera_transform,
                 math::mat4 const& screen_transform,
                 float clip_near,
                 float clip_far)
    : camera_position_(),
      projection_(math::mat4::identity()),
      view_(math::mat4::identity()),
      planes_(6),
      clip_near_(clip_near),
      clip_far_(clip_far) {

  projection_ = math::compute_frustum(
      camera_transform.column(3), screen_transform, clip_near, clip_far);

  math::mat4 view_transform(screen_transform);
  view_transform[12] = 0.f;
  view_transform[13] = 0.f;
  view_transform[14] = 0.f;
  view_transform[15] = 1.f;

  camera_position_ = math::vec3(camera_transform.column(3)[0],
                                camera_transform.column(3)[1],
                                camera_transform.column(3)[2]);

  view_transform =
      scm::math::make_translation(camera_position_) * view_transform;

  view_ = scm::math::inverse(view_transform);

  auto frustum(projection_ * view_);

  //store normals

  //left plane
  planes_[0] = math::vec4(frustum[3] + frustum[0],
                          frustum[7] + frustum[4],
                          frustum[11] + frustum[8],
                          frustum[15] + frustum[12]);

  //right plane
  planes_[1] = math::vec4(frustum[3] - frustum[0],
                          frustum[7] - frustum[4],
                          frustum[11] - frustum[8],
                          frustum[15] - frustum[12]);

  //bottom plane
  planes_[2] = math::vec4(frustum[3] + frustum[1],
                          frustum[7] + frustum[5],
                          frustum[11] + frustum[9],
                          frustum[15] + frustum[13]);

  //top plane
  planes_[3] = math::vec4(frustum[3] - frustum[1],
                          frustum[7] - frustum[5],
                          frustum[11] - frustum[9],
                          frustum[15] - frustum[13]);

  //near plane
  planes_[4] = math::vec4(frustum[3] + frustum[2],
                          frustum[7] + frustum[6],
                          frustum[11] + frustum[10],
                          frustum[15] + frustum[14]);

  //far plane
  planes_[5] = math::vec4(frustum[3] - frustum[2],
                          frustum[7] - frustum[6],
                          frustum[11] - frustum[10],
                          frustum[15] - frustum[14]);

}

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

}
