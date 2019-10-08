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

#ifndef HINGE_CONSTRAINT_HPP
#define HINGE_CONSTRAINT_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/Constraint.hpp>

namespace gua
{
namespace physics
{
/**
 * The hinge constraint connects two rigid bodies or one rigid
 *        body to a fixed axis in worldspace.
 *
 * Two bodies connected with a hinge constraint can only rotate around
 * a given axis. This constraint can be useful to represent doors or wheels
 * rotating around one axis.
 */
class GUA_DLL HingeConstraint : public Constraint
{
  public:
    /**
     * Constructor.
     *
     * Creates a new constraint between one rigid body and a fixed axis in
     * worldspace.
     *
     * \param body_a  The rigid body.
     * \param pivot_a The pivot point in body's local space (axis location).
     * \param axis_a  The hinge axis in body's local space.
     */
    HingeConstraint(RigidBodyNode* body_a, const math::vec3& pivot_a, const math::vec3& axis_a);

    /**
     * Constructor.
     *
     * Creates a new constraint between two rigid bodies.
     *
     * \param body_a  The first rigid body.
     * \param body_b  The second rigid body.
     * \param pivot_a The pivot point in the local space of the first body.
     * \param pivot_b The pivot point in the local space of the second body.
     * \param axis_a  The hinge axis in the local space of the first body.
     * \param axis_b  The hinge axis in the local space of the second body.
     */
    HingeConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::vec3& pivot_a, const math::vec3& pivot_b, const math::vec3& axis_a, const math::vec3& axis_b);

    /**
     * Destructor.
     *
     * Deletes the constraint and frees all associated data.
     */
    virtual ~HingeConstraint() {}

    /**
     * Sets the pivot point of the first rigid body.
     *
     * \param pivot The pivot point in the local space.
     */
    void set_pivot_a(const math::vec3& p);

    /**
     * Sets the pivot point of the second rigid body.
     *
     * \param pivot The pivot point in the local space.
     */
    void set_pivot_b(const math::vec3& p);

    /**
     * Sets the hinge of the first rigid body.
     *
     * \param pivot The hinge axis in the local space.
     */
    void set_axis_a(const math::vec3& a);

    /**
     * Sets the hinge of the second rigid body.
     *
     * \param pivot The hinge axis in the local space.
     */
    void set_axis_b(const math::vec3& a);

    /**
     * Sets lower angular limit.
     *
     * \param limit Angle in radians.
     */
    void set_lower_limit(float limit);

    /**
     * Sets upper angular limit.
     *
     * \param limit Angle in radians.
     */
    void set_upper_limit(float limit);

    /**
     * Sets lower and upper angular limits.
     *
     * \param lower  Lower limit in radians.
     * \param higher Upper limit in radians.
     */
    void set_limit(float lower, float upper);

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

    /**
     * Gets the hinge axis of the first rigid body.
     *
     * \return The hinge axis in the local space.
     */
    const math::vec3& axis_a() const { return axis_a_.first; }

    /**
     * Gets the hinge axis of the second rigid body.
     *
     * \return The hinge axis in the local space.
     */
    const math::vec3& axis_b() const { return axis_b_.first; }

    /**
     * Gets the lower angular limit of the hinge.
     *
     * \return Angle in radians.
     */
    float lower_limit() const { return lower_limit_.first; }

    /**
     * Gets the upper angular limit of the hinge.
     *
     * \return Angle in radians.
     */
    float upper_limit() const { return upper_limit_.first; }

    /**
     * Enables and disables the angular motor.
     *
     * \param enable          If true, motor is enabled.
     * \param target_velocity The target velocity of the rotation.
     * \param max_impulse     The maximum impulse applied to the hinge to
     *                        achieve the target velocity.
     */
    void enable_angular_motor(bool enable, float target_velocity, float max_impulse);

    /**
     * Enables and disables the motor.
     *
     * \param enable          If true, motor is enabled.
     */
    void enable_motor(bool enable);

    bool motor_enabled() const;

    float motor_target_velocity() const;

    float motor_target_impulse() const;

  private:
    virtual void update_constraint();

    Vec3Field pivot_a_;
    Vec3Field pivot_b_;
    Vec3Field axis_a_;
    Vec3Field axis_b_;
    FloatField lower_limit_;
    FloatField upper_limit_;
    BoolField motor_enabled_;
    FloatField motor_target_velocity_;
    FloatField motor_max_impulse_;
};

} // namespace physics
} // namespace gua

#endif // HINGE_CONSTRAINT_HPP
