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

#ifndef SLIDER_CONSTRAINT_HPP
#define SLIDER_CONSTRAINT_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/Constraint.hpp>

namespace gua
{
namespace physics
{
/**
 * The slider constraint connects two rigid bodies or one rigid
 *        body to a fixed axis in worldspace.
 *
 * The constraint allows the body to rotate around one axis and translate
 * along this axis.
 */
class GUA_DLL SliderConstraint : public Constraint
{
  public:
    /**
     * Constructor.
     *
     * Creates a new constraint between one rigid body and a fixed axis in
     * worldspace.
     *
     * \param body_a  The rigid body.
     * \param frame_a The body's frame.
     */
    SliderConstraint(RigidBodyNode* body_a, const math::mat4& frame_a);

    /**
     * Constructor.
     *
     * Creates a new constraint between two rigid bodies.
     *
     * \param body_a  The first rigid body.
     * \param body_b  The second rigid body.
     * \param frame_a The frame of the first body.
     * \param frame_b The frame of the second body.
     */
    SliderConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::mat4& frame_a, const math::mat4& frame_b);

    /**
     * Destructor.
     *
     * Deletes the constraint and frees all associated data.
     */
    virtual ~SliderConstraint() {}

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

    ///@{
    /**
     * Sets lower and upper linear limits.
     *
     * \param limit Maximum and minimum distance between pivot points.
     */
    void set_lower_linear_limit(float limit);
    void set_upper_linear_limit(float limit);
    ///@}

    /**
     * Sets lower and upper linear limits.
     *
     * \param lower  Lower limit.
     * \param higher Upper limit.
     */
    void set_linear_limit(float lower, float upper);

    ///@{
    /**
     * Sets lower and upper angular limits.
     *
     * \param limit Angle in radians.
     */
    void set_lower_angular_limit(float limit);
    void set_upper_angular_limit(float limit);
    ///@}

    /**
     * Sets lower and upper angular limits.
     *
     * \param lower  Lower limit in radians.
     * \param higher Upper limit in radians.
     */
    void set_angular_limit(float lower, float upper);

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
     * Gets the lower linear limit of the slider.
     *
     * \return Lower limit
     */
    float lower_linear_limit() const { return lower_linear_limit_.first; }

    /**
     * Gets the upper linear limit of the slider.
     *
     * \return Upper limit
     */
    float upper_linear_limit() const { return upper_linear_limit_.first; }

    /**
     * Gets the lower angular limit of the slider.
     *
     * \return Angle in radians.
     */
    float lower_angular_limit() const { return lower_angular_limit_.first; }

    /**
     * Gets the upper angular limit of the slider.
     *
     * \return Angle in radians.
     */
    float upper_angular_limit() const { return upper_angular_limit_.first; }

  private:
    virtual void update_constraint();

    Mat4Field frame_a_;
    Mat4Field frame_b_;
    FloatField lower_linear_limit_;
    FloatField upper_linear_limit_;
    FloatField lower_angular_limit_;
    FloatField upper_angular_limit_;
};

} // namespace physics
} // namespace gua

#endif // SLIDER_CONSTRAINT_HPP
