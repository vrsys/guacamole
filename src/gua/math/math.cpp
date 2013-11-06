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

// header
#include <gua/math/math.hpp>

// external headers
#include <iostream>
#include <sstream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

math::mat4 const math::compute_frustum(math::vec4 const& eye_position,
                                       math::mat4 const& screen_transform,
                                       float near_plane,
                                       float far_plane) {

  math::vec4 relative_eye_position(scm::math::inverse(screen_transform) *
                                   eye_position);

  float d(relative_eye_position[2]);
  float ox(-relative_eye_position[0]);
  float oy(-relative_eye_position[1]);

  math::mat4 frustum(math::mat4::identity());

  frustum[0] = 2 * d;
  frustum[5] = 2 * d;
  frustum[8] = 2 * ox;
  frustum[9] = 2 * oy;
  frustum[10] = (-near_plane - far_plane) / (far_plane - near_plane);
  frustum[11] = -1.f;
  frustum[14] = -2 * near_plane * far_plane / (far_plane - near_plane);
  frustum[15] = 0.f;

  return frustum;
}

////////////////////////////////////////////////////////////////////////////////

math::mat4 const math::mat_ai_to_scm(aiMatrix4x4 const& ai_mat) {

  math::mat4 scm_mat;

  for (unsigned i(0); i < 16; ++i) {
    unsigned column(i % 4);
    unsigned row(i / 4);
    unsigned index(column * 4 + row);
    scm_mat[index] = *ai_mat[i];
  }

  //    for (unsigned i(0); i<16; ++i) {
  //        scm_mat[i] = *ai_mat[i];
  //    }

  return scm_mat;
}

////////////////////////////////////////////////////////////////////////////////

std::tuple<float,float,float> barycentric(math::vec3 const& a,
                                          math::vec3 const& b,
                                          math::vec3 const& c,
                                          math::vec3 const& p) {
  auto ap = a-p;
  auto bp = b-p;
  auto cp = c-p;

  auto area = scm::math::length(scm::math::cross(a-b, a-c));
  auto a1   = scm::math::length(scm::math::cross(bp, cp)) / area;
  auto a2   = scm::math::length(scm::math::cross(cp, ap)) / area;
  auto a3   = 1.0 - a1 - a2;

  return std::make_tuple(a1,a2,a3);
}

}
