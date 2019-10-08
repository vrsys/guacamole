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

#ifndef GUA_COLLISION_SHAPE_HPP
#define GUA_COLLISION_SHAPE_HPP

#include <gua/platform.hpp>

class btCollisionShape;
class btCompoundShape;
class btTransform;

namespace gua
{
namespace physics
{
/**
 * A class representing collision shapes.
 *
 * This abstract class provides an interface for collision shapes that can be
 * shared among multiple rigid bodies.
 */
class GUA_DLL CollisionShape
{
    friend class RigidBodyNode;

  public:
    /**
     * Constructor.
     *
     * Creates a new collision shape.
     *
     * \param static_shape  The shape supports static rigid bodies.
     * \param dynamic_shape The shape supports dynamic rigid bodies.
     * \param identical_shape_contructor The shape constructors are the same.
     */
    explicit CollisionShape(bool static_shape, bool dynamic_shape, bool identical_shape_contructor)
        : is_static_shape_(static_shape), is_dynamic_shape_(dynamic_shape), has_identical_shape_constructor_(identical_shape_contructor)
    {
    }

    /**
     * Destructor.
     *
     * Deletes the collision shape and frees all associated data.
     */
    virtual ~CollisionShape() {}

    /**
     * Check whether the collision shape can be used in static
     *       rigid bodies.
     *
     * \return True if the shape supports static rigid bodies.
     */
    bool is_static_shape() const { return is_static_shape_; }

    void set_is_static_shape(bool is_static_shape) { is_static_shape_ = is_static_shape; }

    /**
     * Check whether the collision shape can be used in dynamic
     *       rigid bodies.
     *
     * \return True if the shape supports dynamic rigid bodies.
     */
    bool is_dynamic_shape() const { return is_dynamic_shape_; }

    void set_is_dynamic_shape(bool is_dynamic_shape) { is_dynamic_shape_ = is_dynamic_shape; }

    /**
     * Check whether the shape constructors (construct_dynamic() and
     *       construct_static()) generate identical collision shapes.
     *       It makes it possible to change a static rigid body to a dynamic
     *       one on fly without recreating its compound shape when this
     *       function returns true for all associated collsion shapes.
     *
     * \return True if the shape constructors are the same.
     */
    bool has_identical_shape_constructor() const { return has_identical_shape_constructor_; }

    void set_has_identical_shape_constructor(bool has_identical_shape_constructor) { has_identical_shape_constructor_ = has_identical_shape_constructor; }

  protected:
    /**
     * Fills a bullet's compound shape of dynamic rigid bodies.
     *
     * This is an internal method that is used by rigid body traversal in
     * order to construct a compound shape. The method can be called in the
     * traversal only if is_dynamic_shape() returns true.
     *
     * \param bullet_shape   The rigid body's compound shape.
     * \param base_transform The transformation of the collision node
     *                       associated with this collision shape.
     * \sa construct_static()
     */
    virtual void construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform) = 0;

    /**
     * Sets a bullet's collision shape for static rigid bodies.
     *
     * This is an internal method that is used by rigid body traversal in
     * order to construct a single collision shape. The method can be called in
     * the traversal only if is_static_shape() returns true.
     *
     * \return   New rigid body's collision shape.
     * \sa construct_dynamic()
     */
    virtual btCollisionShape* construct_static() = 0;

    bool is_static_shape_;
    bool is_dynamic_shape_;
    bool has_identical_shape_constructor_;
};

} // namespace physics
} // namespace gua

#endif // GUA_COLLISION_SHAPE_HPP
