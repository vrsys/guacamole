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
#include <gua/physics/SliderConstraint.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/physics/PhysicsUtils.hpp>

// external headers
#include <mutex>
using std::lock_guard;

#include <btBulletDynamicsCommon.h>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

SliderConstraint::SliderConstraint(RigidBodyNode* body_a, const math::mat4& frame_a) : Constraint(body_a, nullptr), frame_a_(frame_a, false), frame_b_(math::mat4::identity(), false)
{
    auto ct = new btSliderConstraint(*(body_a->get_bullet_rigid_body()), math::mat4_to_btTransform(frame_a), true);
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    lower_linear_limit_ = std::make_pair(ct->getLowerLinLimit(), false);
    upper_linear_limit_ = std::make_pair(ct->getUpperLinLimit(), false);
    lower_angular_limit_ = std::make_pair(ct->getLowerAngLimit(), false);
    upper_angular_limit_ = std::make_pair(ct->getUpperAngLimit(), false);
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

SliderConstraint::SliderConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::mat4& frame_a, const math::mat4& frame_b)
    : Constraint(body_a, body_b), frame_a_(frame_a, false), frame_b_(frame_b, false)
{
    auto ct = new btSliderConstraint(*(body_a->get_bullet_rigid_body()), *(body_b->get_bullet_rigid_body()), math::mat4_to_btTransform(frame_a), math::mat4_to_btTransform(frame_b), false);
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    lower_linear_limit_ = std::make_pair(ct->getLowerLinLimit(), false);
    upper_linear_limit_ = std::make_pair(ct->getUpperLinLimit(), false);
    lower_angular_limit_ = std::make_pair(ct->getLowerAngLimit(), false);
    upper_angular_limit_ = std::make_pair(ct->getUpperAngLimit(), false);
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_frame_a(const math::mat4& frame)
{
    if(frame_a_.first == frame)
        return;
    lock_guard<SpinLock> lk(lock_);
    frame_a_ = std::make_pair(frame, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_frame_b(const math::mat4& frame)
{
    if(frame_b_.first == frame)
        return;
    lock_guard<SpinLock> lk(lock_);
    frame_b_ = std::make_pair(frame, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_lower_linear_limit(float limit)
{
    if(lower_linear_limit_.first == limit)
        return;
    lock_guard<SpinLock> lk(lock_);
    lower_linear_limit_ = std::make_pair(limit, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_upper_linear_limit(float limit)
{
    if(upper_linear_limit_.first == limit)
        return;
    lock_guard<SpinLock> lk(lock_);
    upper_linear_limit_ = std::make_pair(limit, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_linear_limit(float lower, float upper)
{
    if(lower_linear_limit_.first == lower && upper_linear_limit_.first == upper)
        return;
    lock_guard<SpinLock> lk(lock_);
    lower_linear_limit_ = std::make_pair(lower, true);
    upper_linear_limit_ = std::make_pair(upper, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_lower_angular_limit(float limit)
{
    if(lower_angular_limit_.first == limit)
        return;
    lock_guard<SpinLock> lk(lock_);
    lower_angular_limit_ = std::make_pair(limit, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_upper_angular_limit(float limit)
{
    if(upper_angular_limit_.first == limit)
        return;
    lock_guard<SpinLock> lk(lock_);
    upper_angular_limit_ = std::make_pair(limit, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void SliderConstraint::set_angular_limit(float lower, float upper)
{
    if(lower_angular_limit_.first == lower && upper_angular_limit_.first == upper)
        return;
    lock_guard<SpinLock> lk(lock_);
    lower_angular_limit_ = std::make_pair(lower, true);
    upper_angular_limit_ = std::make_pair(upper, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void SliderConstraint::update_constraint()
{
    auto ct = static_cast<btSliderConstraint*>(ct_);

    if(!body_b_ && frame_b_.second)
    {
        frame_b_.second = false;
    }
    if(frame_a_.second || frame_b_.second)
    {
        ct->setFrames(math::mat4_to_btTransform(frame_a_.first), math::mat4_to_btTransform(frame_b_.first));
        frame_a_.second = false;
        frame_b_.second = false;
    }
    if(lower_linear_limit_.second)
    {
        ct->setLowerLinLimit(lower_linear_limit_.first);
        lower_linear_limit_.second = false;
    }
    if(upper_linear_limit_.second)
    {
        ct->setUpperLinLimit(upper_linear_limit_.first);
        upper_linear_limit_.second = false;
    }
    if(lower_angular_limit_.second)
    {
        ct->setLowerAngLimit(lower_angular_limit_.first);
        lower_angular_limit_.second = false;
    }
    if(upper_angular_limit_.second)
    {
        ct->setUpperAngLimit(upper_angular_limit_.first);
        upper_angular_limit_.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
