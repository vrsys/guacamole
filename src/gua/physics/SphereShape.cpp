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
#include <gua/physics/SphereShape.hpp>
#include <gua/memory.hpp>

// external headers
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <BulletCollision/CollisionShapes/btSphereShape.h>

namespace gua
{
namespace physics
{
SphereShape::SphereShape(float radius) : CollisionShape(true, true, true), shape_(gua::make_unique<btSphereShape>(radius)), radius_(radius) {}

void SphereShape::set_radius(float radius)
{
    radius_ = radius;
    shape_ = gua::make_unique<btSphereShape>(radius_);
}

void SphereShape::construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform) { bullet_shape->addChildShape(base_transform, shape_.get()); }

btCollisionShape* SphereShape::construct_static() { return shape_.get(); }

} // namespace physics
} // namespace gua
