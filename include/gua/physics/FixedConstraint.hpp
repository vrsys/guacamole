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
#ifndef FIXED_CONSTRAINT_HPP
#define FIXED_CONSTRAINT_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/Constraint.hpp>

namespace gua
{
namespace physics
{
/**
 * The fixed constraint rigidly connects two rigid bodies or one rigid
 *        body to a fixed position in worldspace.
 *
 * The constraint restricts mutual rotations and translations between rigid
 * bodies or between a rigid body and static environment.
 */
class GUA_DLL FixedConstraint : public Constraint
{
  public:
    /**
     * Constructor.
     *
     * Creates a new fixed constraint between a rigid body and static
     * environment.
     *
     * \param body_a           The rigid body.
     * \param frame_a          The body's frame.
     * \param lock_rotation    Suppress rotation of the rigid body.
     * \param lock_translation Suppress translation of the rigid body.
     */
    FixedConstraint(RigidBodyNode* body_a, const math::mat4& frame_a, bool lock_rotation = true, bool lock_translation = true);

    /**
     * Constructor.
     *
     * Creates a new fixed constraint between two rigid bodies.
     *
     * \param body_a           The first rigid body.
     * \param body_b           The second rigid body.
     * \param frame_a          The frame of the first body.
     * \param frame_b          The frame of the second body.
     * \param lock_rotation    Suppress mutual rotation of the rigid bodies.
     * \param lock_translation Suppress mutual translation of the rigid bodies.
     */
    FixedConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::mat4& frame_a, const math::mat4& frame_b, bool lock_rotation = true, bool lock_translation = true);

    /**
     * Destructor.
     *
     * Deletes the constraint and frees all associated data.
     */
    virtual ~FixedConstraint() {}

    /**
     * Sets the spatial frame of the first rigid body.
     *
     * \param frame The frame
     */
    void set_frame_a(const math::mat4& frame);

    /**
     * Sets the spatial frame of the second rigid body.
     *
     * \param frame The frame
     */
    void set_frame_b(const math::mat4& frame);

    /**
     * Allows or disallows mutual rotation
     *
     * \param lock If true, rotation is locked.
     */
    void lock_rotation(bool lock);

    /**
     * Allows or disallows mutual translation
     *
     * \param lock If true, translation is locked.
     */
    void lock_translation(bool lock);

    /**
     * Gets the spatial frame of the first rigid body.
     *
     * \return The frame
     */
    const math::mat4& frame_a() const { return frame_a_.first; }

    /**
     * Gets the spatial frame of the second rigid body.
     *
     * \return The frame
     */
    const math::mat4& frame_b() const { return frame_b_.first; }

    /**
     * Tests whether the constraint does not allow rotation
     *
     * \return True if rotation is locked.
     */
    bool is_rotation_locked() const { return lock_rotation_.first; }

    /**
     * Tests whether the constraint does not allow translation
     *
     * \return True if translation is locked.
     */
    bool is_translation_locked() const { return lock_translation_.first; }

  private:
    virtual void update_constraint();

    Mat4Field frame_a_;
    Mat4Field frame_b_;
    BoolField lock_rotation_;
    BoolField lock_translation_;
};

} // namespace physics
} // namespace gua

#endif // FIXED_CONSTRAINT_HPP
