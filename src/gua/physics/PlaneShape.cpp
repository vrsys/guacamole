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
#include <gua/physics/PlaneShape.hpp>

// guacamole headers
#include <gua/physics/PhysicsUtils.hpp>

// external headers
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <stdexcept>
#include <gua/memory.hpp>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

PlaneShape::PlaneShape(float x_normal, float y_normal, float z_normal, float plane_constant)
    : CollisionShape(true, false, false), shape_(gua::make_unique<btStaticPlaneShape>(btVector3(x_normal, y_normal, z_normal), plane_constant)), normal_(x_normal, y_normal, z_normal),
      plane_constant_(plane_constant)
{
}

void PlaneShape::construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform)
{
    throw std::runtime_error("Plane shapes cannot take part in dynamical "
                             "simulations.");
}

void PlaneShape::set_normal(math::vec3 const& normal)
{
    normal_ = normal;
    shape_ = gua::make_unique<btStaticPlaneShape>(math::vec3_to_btVector3(normal_), plane_constant_);
}

void PlaneShape::set_plane_constant(float plane_constant)
{
    plane_constant_ = plane_constant;
    shape_ = gua::make_unique<btStaticPlaneShape>(math::vec3_to_btVector3(normal_), plane_constant_);
}

btCollisionShape* PlaneShape::construct_static() { return shape_.get(); }

} // namespace physics
} // namespace gua
