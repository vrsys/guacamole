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

#ifndef POINT_2_POINT_CONSTRAINT_HPP
#define POINT_2_POINT_CONSTRAINT_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/Constraint.hpp>

namespace gua
{
namespace physics
{
/**
 * The point-to-point constraint connects two rigid bodies or one rigid
 *        body and a fixed point in worldspace.
 *
 * This constraint represents a ball socket joint.
 */
class GUA_DLL Point2PointConstraint : public Constraint
{
  public:
    /**
     * Constructor.
     *
     * Creates a new constraint between one rigid body and a fixed point in
     * worldspace.
     *
     * \param body_a  The rigid body.
     * \param pivot_a The pivot point in body's local space.
     */
    Point2PointConstraint(RigidBodyNode* body_a, const math::vec3& pivot_a);

    /**
     * Constructor.
     *
     * Creates a new constraint between two rigid bodies. The constraint limits
     * the translation so that the local pivot points of 2 rigidbodies match
     * in worldspace.
     *
     * \param body_a  The first rigid body.
     * \param body_b  The second rigid body.
     * \param pivot_a The pivot point in the local space of the first body.
     * \param pivot_b The pivot point in the local space of the second body.
     */
    Point2PointConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::vec3& pivot_a, const math::vec3& pivot_b);

    /**
     * Destructor.
     *
     * Deletes the constraint and frees all associated data.
     */
    virtual ~Point2PointConstraint() {}

    /**
     * Sets the pivot point of the first rigid body.
     *
     * \param pivot The pivot point in the local space.
     */
    void set_pivot_a(const math::vec3& pivot);

    /**
     * Sets the pivot point of the second rigid body.
     *
     * \param pivot The pivot point in the local space.
     */
    void set_pivot_b(const math::vec3& pivot);

    /**
     * Gets the pivot point of the first rigid body.
     *
     * \return The pivot point in the local space.
     */
    const math::vec3& pivot_a() const { return pivot_a_.first; }

    /**
     * Gets the pivot point of the second rigid body.
     *
     * \return The pivot point in the local space.
     */
    const math::vec3& pivot_b() const { return pivot_b_.first; }

  private:
    virtual void update_constraint();

    Vec3Field pivot_a_;
    Vec3Field pivot_b_;
};

} // namespace physics
} // namespace gua

#endif // POINT_2_POINT_CONSTRAINT_HPP
