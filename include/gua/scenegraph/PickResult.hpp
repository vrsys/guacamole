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

#ifndef GUA_PICK_RESULT_HPP
#define GUA_PICK_RESULT_HPP

// guacamole headers
#include <gua/math/math.hpp>


namespace gua {

class Node;

struct PickResult {

  enum Options { PICK_ALL                 = 0,
                 PICK_ONLY_FIRST_OBJECT   = 1<<1,
                 PICK_ONLY_FIRST_FACE     = 1<<2,
                 GET_POSITIONS            = 1<<3,
                 GET_WORLD_POSITIONS      = 1<<4,
                 GET_NORMALS              = 1<<5,
                 GET_WORLD_NORMALS        = 1<<6,
                 INTERPOLATE_NORMALS      = 1<<7,
                 GET_TEXTURE_COORDS       = 1<<8
               };

  PickResult(float d, Node* o,
             math::vec3 const& p, math::vec3 const& wp,
             math::vec3 const& n, math::vec3 const& wn,
             math::vec2 const& t)
    : distance(d), object(o),
      position(p), world_position(wp),
      normal(n), world_normal(wn),
      texture_coords(t) {}

  float                distance;
  Node*                object;
  mutable math::vec3   position;
  mutable math::vec3   world_position;
  mutable math::vec3   normal;
  mutable math::vec3   world_normal;
  mutable math::vec2   texture_coords;

  bool operator<(PickResult const& lhs) const {
    return distance < lhs.distance;
  }
};

}

#endif  // GUA_PICK_RESULT_HPP
