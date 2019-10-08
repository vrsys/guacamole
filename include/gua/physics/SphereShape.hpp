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
#ifndef GUA_SPHERE_SHAPE_HPP
#define GUA_SPHERE_SHAPE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/CollisionShape.hpp>
#include <memory>

class btSphereShape;

namespace gua
{
namespace physics
{
/**
 * A class representing a sphere-shaped collision shape.
 *
 * This class is a sphere primitive around the origin with the given radius.
 * The sphere shape can be used for both static and dynamic rigid bodies.
 */
class GUA_DLL SphereShape : public CollisionShape
{
  public:
    /**
     * Constructor.
     *
     * Creates a new sphere shape with the given radius.
     *
     * \param radius The radius of the sphere.
     */
    SphereShape(float radius);

    inline float get_radius() const { return radius_; }

    void set_radius(float radius);

  private:
    void construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform) override;

    btCollisionShape* construct_static() override;

    std::unique_ptr<btSphereShape> shape_;
    float radius_;
};

} // namespace physics
} // namespace gua

#endif // GUA_SPHERE_SHAPE_HPP
