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

#ifndef GUA_CYLINDER_SHAPE_HPP
#define GUA_CYLINDER_SHAPE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/CollisionShape.hpp>
#include <gua/physics/PhysicsUtils.hpp>

class btCylinderShape;

namespace gua {
namespace physics {

/**
 * A class representing a cylinder-shaped collision shape.
 *
 * This class is a cylinder primitive around the origin, its central axis
 * aligned with Y axis of given half-extents vector.
 * The cylinder shape can be used for both static and dynamic rigid bodies.
 */
class GUA_DLL CylinderShape : public CollisionShape {
 public:

  /**
   * Constructor.
   *
   * Creates a new cylinder shape with the given vector containg half extents
   * for each axis.
   *
   * \param vec The vector with the half-extents. Y is the central axis of
   *            the cylinder
   */
  CylinderShape(const math::vec3& half_extents);

  /**
   * Destructor.
   *
   * Deletes the cylinder shape and frees all associated data.
   */
  virtual ~CylinderShape();

  math::vec3 const& get_half_extents() const;

  void set_half_extents(math::vec3 const& half_extents);

 private:

  virtual void construct_dynamic(btCompoundShape* bullet_shape,
                                 const btTransform& base_transform);

  virtual btCollisionShape* construct_static();

  btCylinderShape* shape_;
  math::vec3 half_extents_;
};

}
}

#endif  // GUA_CYLINDER_SHAPE_HPP
