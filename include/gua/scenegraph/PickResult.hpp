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
#include <gua/platform.hpp>
#include <gua/math/math.hpp>


namespace gua {

class Node;

struct GUA_DLL PickResult {

  enum Options {
                 /// A PickResult is returned for each hit Node along a RayNode's
                 /// length. This is the default value for picking.
                 PICK_ALL                 = 0,
                 /// Only the first hit Node is used as a PickResult. The
                 /// intersection process is terminated after hitting one Node.
                 PICK_ONLY_FIRST_OBJECT   = 1<<1,
                 /// Only the first face of each hit Node is used as a
                 /// PickResult.
                 PICK_ONLY_FIRST_FACE     = 1<<2,
                 /// If set, the positions of all intersection points in the hit
                 /// Nodes' object coordinates are written to the respective
                 /// PickResult.
                 GET_POSITIONS            = 1<<3,
                 GET_WORLD_POSITIONS      = 1<<4,
                 GET_NORMALS              = 1<<5,
                 GET_WORLD_NORMALS        = 1<<6,
                 INTERPOLATE_NORMALS      = 1<<7,
                 GET_TEXTURE_COORDS       = 1<<8
               };

  /**
   * Constructor.
   *
   * This constructs a PickResult with the given parameters. A PickResult is
   * instantiated for each successful intersection of a RayNode with a
   * GeometryNode.
   *
   * \param d   The distance between the RayNode's origin and the
   *            intersection point.
   * \param o   The Node beeing hit.
   * \param p   The hit's position in the hit Node's object coordinates.
   * \param wp  The hit's position in world coordinates.
   * \param n   The surface normal at the hit's position the hit Node's object
   *            coordinates.
   * \param wn  The surface normal at the hit's position in world coordinates.
   * \param t   The hit object's texture coordinates at the hit's position.
   */
  PickResult(float d, Node* o,
             math::vec3 const& p, math::vec3 const& wp,
             math::vec3 const& n, math::vec3 const& wn,
             math::vec2 const& t)
    : distance(d), object(o),
      position(p), world_position(wp),
      normal(n), world_normal(wn),
      texture_coords(t) {}

  /// The distance between a RayNode's origin and the intersection point.
  float                distance;
  /// The Node beeing hit.
  Node*                object;
  /// The Node beeing hit.
  mutable math::vec3   position;
  /// The hit's position in the hit Node's object coordinates.
  mutable math::vec3   world_position;
  /// The surface normal at the hit's position the hit Node's object coordinates.
  mutable math::vec3   normal;
  /// The surface normal at the hit's position in world coordinates.
  mutable math::vec3   world_normal;
  /// The hit object's texture coordinates at the hit's position.
  mutable math::vec2   texture_coords;

  bool operator<(PickResult const& lhs) const {
    return distance < lhs.distance;
  }
};

}

#endif  // GUA_PICK_RESULT_HPP
