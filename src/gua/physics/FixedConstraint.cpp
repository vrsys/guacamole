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
#include <gua/physics/FixedConstraint.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/physics/PhysicsUtils.hpp>

// external headers
#include <btBulletDynamicsCommon.h>
#include <mutex>

using std::lock_guard;

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

FixedConstraint::FixedConstraint(RigidBodyNode* body_a, const math::mat4& frame_a, bool lock_rotation, bool lock_translation)
    : Constraint(body_a, nullptr), frame_a_(frame_a, false), frame_b_(math::mat4::identity(), false), lock_rotation_(lock_rotation, false), lock_translation_(lock_translation, false)
{
    auto ct = new btGeneric6DofConstraint(*(body_a->get_bullet_rigid_body()), math::mat4_to_btTransform(frame_a), true);
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    ct->setLinearUpperLimit(btVector3(0.f, 0.f, 0.f));
    ct->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
    if(lock_translation)
        ct->setLinearLowerLimit(btVector3(0.f, 0.f, 0.f));
    else
        ct->setLinearLowerLimit(btVector3(1.f, 1.f, 1.f));
    if(lock_rotation)
        ct->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
    else
        ct->setAngularLowerLimit(btVector3(1.f, 1.f, 1.f));
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

FixedConstraint::FixedConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::mat4& frame_a, const math::mat4& frame_b, bool lock_rotation, bool lock_translation)
    : Constraint(body_a, body_b), frame_a_(frame_a, false), frame_b_(frame_b, false), lock_rotation_(lock_rotation, false), lock_translation_(lock_translation, false)
{
    auto ct = new btGeneric6DofConstraint(*(body_a->get_bullet_rigid_body()), *(body_b->get_bullet_rigid_body()), math::mat4_to_btTransform(frame_a), math::mat4_to_btTransform(frame_b), false);
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    ct->setLinearUpperLimit(btVector3(0.f, 0.f, 0.f));
    ct->setAngularUpperLimit(btVector3(0.f, 0.f, 0.f));
    if(lock_translation)
        ct->setLinearLowerLimit(btVector3(0.f, 0.f, 0.f));
    else
        ct->setLinearLowerLimit(btVector3(1.f, 1.f, 1.f));
    if(lock_rotation)
        ct->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
    else
        ct->setAngularLowerLimit(btVector3(1.f, 1.f, 1.f));
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

void FixedConstraint::set_frame_a(const math::mat4& frame)
{
    if(frame_a_.first == frame)
        return;
    lock_guard<SpinLock> lk(lock_);
    frame_a_ = std::make_pair(frame, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void FixedConstraint::set_frame_b(const math::mat4& frame)
{
    if(frame_b_.first == frame)
        return;
    lock_guard<SpinLock> lk(lock_);
    frame_b_ = std::make_pair(frame, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void FixedConstraint::lock_rotation(bool lock)
{
    if(lock_rotation_.first == lock)
        return;
    lock_guard<SpinLock> lk(lock_);
    lock_rotation_ = std::make_pair(lock, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void FixedConstraint::lock_translation(bool lock)
{
    if(lock_translation_.first == lock)
        return;
    lock_guard<SpinLock> lk(lock_);
    lock_translation_ = std::make_pair(lock, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void FixedConstraint::update_constraint()
{
    auto ct = static_cast<btGeneric6DofConstraint*>(ct_);

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
    if(lock_translation_.second)
    {
        if(lock_translation_.first)
            ct->setLinearLowerLimit(btVector3(0.f, 0.f, 0.f));
        else
            ct->setLinearLowerLimit(btVector3(1.f, 1.f, 1.f));
        lock_translation_.second = false;
    }
    if(lock_rotation_.second)
    {
        if(lock_rotation_.first)
            ct->setAngularLowerLimit(btVector3(0.f, 0.f, 0.f));
        else
            ct->setAngularLowerLimit(btVector3(1.f, 1.f, 1.f));
        lock_rotation_.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
