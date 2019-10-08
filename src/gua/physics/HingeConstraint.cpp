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
#include <gua/physics/HingeConstraint.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/physics/PhysicsUtils.hpp>

// external headers
#include <mutex>
#include <btBulletDynamicsCommon.h>

using std::lock_guard;

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

HingeConstraint::HingeConstraint(RigidBodyNode* body_a, const math::vec3& pivot_a, const math::vec3& axis_a)
    : Constraint(body_a, nullptr), pivot_a_(pivot_a, false), pivot_b_(math::vec3(0.f), false), axis_a_(axis_a, false), axis_b_(math::vec3(0.f), false)
{
    auto ct = new btHingeConstraint(*(body_a->get_bullet_rigid_body()), math::vec3_to_btVector3(pivot_a), math::vec3_to_btVector3(axis_a));
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    lower_limit_ = std::make_pair(ct->getLowerLimit(), false);
    upper_limit_ = std::make_pair(ct->getUpperLimit(), false);
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

HingeConstraint::HingeConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::vec3& pivot_a, const math::vec3& pivot_b, const math::vec3& axis_a, const math::vec3& axis_b)
    : Constraint(body_a, body_b), pivot_a_(pivot_a, false), pivot_b_(pivot_b, false), axis_a_(axis_a, false), axis_b_(axis_b, false)
{
    auto ct = new btHingeConstraint(*(body_a->get_bullet_rigid_body()),
                                    *(body_b->get_bullet_rigid_body()),
                                    math::vec3_to_btVector3(pivot_a),
                                    math::vec3_to_btVector3(pivot_b),
                                    math::vec3_to_btVector3(axis_a),
                                    math::vec3_to_btVector3(axis_b));
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    lower_limit_ = std::make_pair(ct->getLowerLimit(), false);
    upper_limit_ = std::make_pair(ct->getUpperLimit(), false);
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_pivot_a(const math::vec3& p)
{
    if(pivot_a_.first == p)
        return;
    lock_guard<SpinLock> lk(lock_);
    pivot_a_ = std::make_pair(p, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_pivot_b(const math::vec3& p)
{
    if(pivot_b_.first == p)
        return;
    lock_guard<SpinLock> lk(lock_);
    pivot_b_ = std::make_pair(p, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_axis_a(const math::vec3& a)
{
    if(axis_a_.first == a)
        return;
    lock_guard<SpinLock> lk(lock_);
    axis_a_ = std::make_pair(a, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_axis_b(const math::vec3& a)
{
    if(axis_b_.first == a)
        return;
    lock_guard<SpinLock> lk(lock_);
    axis_b_ = std::make_pair(a, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_lower_limit(float limit)
{
    if(lower_limit_.first == limit)
        return;
    lock_guard<SpinLock> lk(lock_);
    lower_limit_ = std::make_pair(limit, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_upper_limit(float limit)
{
    if(upper_limit_.first == limit)
        return;
    lock_guard<SpinLock> lk(lock_);
    upper_limit_ = std::make_pair(limit, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::set_limit(float lower, float upper)
{
    if(lower_limit_.first == lower && upper_limit_.first == upper)
        return;
    lock_guard<SpinLock> lk(lock_);
    lower_limit_ = std::make_pair(lower, true);
    upper_limit_ = std::make_pair(upper, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::enable_angular_motor(bool enable, float target_velocity, float max_impulse)
{
    if(motor_enabled_.first == enable && motor_target_velocity_.first == target_velocity && motor_max_impulse_.first == max_impulse)
        return;
    lock_guard<SpinLock> lk(lock_);
    motor_enabled_ = std::make_pair(enable, true);
    motor_target_velocity_ = std::make_pair(target_velocity, true);
    motor_max_impulse_ = std::make_pair(max_impulse, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void HingeConstraint::enable_motor(bool enable)
{
    if(motor_enabled_.first == enable)
        return;
    lock_guard<SpinLock> lk(lock_);
    motor_enabled_ = std::make_pair(enable, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

bool HingeConstraint::motor_enabled() const { return motor_enabled_.first; }

////////////////////////////////////////////////////////////////////////////////

float HingeConstraint::motor_target_velocity() const { return motor_target_velocity_.first; }

////////////////////////////////////////////////////////////////////////////////

float HingeConstraint::motor_target_impulse() const { return motor_max_impulse_.first; }

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void HingeConstraint::update_constraint()
{
    auto ct = static_cast<btHingeConstraint*>(ct_);

    if(!body_b_ && (axis_b_.second || pivot_b_.second))
    {
        pivot_b_.second = false;
        axis_b_.second = false;
    }
    if(axis_a_.second || axis_b_.second || pivot_a_.second || pivot_b_.second)
    {
        btTransform frame_a, frame_b;
        btVector3 axisInA, axisInB, rbAxisA1, rbAxisA2;
        frame_a.getOrigin() = math::vec3_to_btVector3(pivot_a_.first);
        axisInA = math::vec3_to_btVector3(axis_a_.first);
        auto rb_a_tr = body_a_->get_bullet_rigid_body()->getCenterOfMassTransform();
        if(body_b_)
        {
            rbAxisA1 = rb_a_tr.getBasis().getColumn(0);
            btScalar projection = axisInA.dot(rbAxisA1);
            if(projection >= 1.0f - SIMD_EPSILON)
            {
                rbAxisA1 = -rb_a_tr.getBasis().getColumn(2);
                rbAxisA2 = rb_a_tr.getBasis().getColumn(1);
            }
            else if(projection <= -1.0f + SIMD_EPSILON)
            {
                rbAxisA1 = rb_a_tr.getBasis().getColumn(2);
                rbAxisA2 = rb_a_tr.getBasis().getColumn(1);
            }
            else
            {
                rbAxisA2 = axisInA.cross(rbAxisA1);
                rbAxisA1 = rbAxisA2.cross(axisInA);
            }
            frame_b.getOrigin() = math::vec3_to_btVector3(pivot_b_.first);
            axisInB = math::vec3_to_btVector3(axis_b_.first);
        }
        else
        {
            btPlaneSpace1(axisInA, rbAxisA1, rbAxisA2);
            frame_b.getOrigin() = rb_a_tr(math::vec3_to_btVector3(pivot_a_.first));
            axisInB = rb_a_tr.getBasis() * axisInA;
        }
        frame_a.getBasis().setValue(rbAxisA1.getX(), rbAxisA2.getX(), axisInA.getX(), rbAxisA1.getY(), rbAxisA2.getY(), axisInA.getY(), rbAxisA1.getZ(), rbAxisA2.getZ(), axisInA.getZ());
        btQuaternion rotationArc = shortestArcQuat(axisInA, axisInB);
        btVector3 rbAxisB1 = quatRotate(rotationArc, rbAxisA1);
        btVector3 rbAxisB2 = axisInB.cross(rbAxisB1);
        frame_b.getBasis().setValue(rbAxisB1.getX(), rbAxisB2.getX(), axisInB.getX(), rbAxisB1.getY(), rbAxisB2.getY(), axisInB.getY(), rbAxisB1.getZ(), rbAxisB2.getZ(), axisInB.getZ());
        ct->setFrames(frame_a, frame_b);
        pivot_a_.second = false;
        pivot_b_.second = false;
        axis_a_.second = false;
        axis_b_.second = false;
    }
    if(lower_limit_.second || upper_limit_.second)
    {
        ct->setLimit(lower_limit_.first, upper_limit_.first);
        lower_limit_.second = false;
        upper_limit_.second = false;
    }
    if(motor_enabled_.second || motor_target_velocity_.second || motor_max_impulse_.second)
    {
        ct->enableAngularMotor(motor_enabled_.first, motor_target_velocity_.first, motor_max_impulse_.first);
        motor_enabled_.second = false;
        motor_target_velocity_.second = false;
        motor_max_impulse_.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
