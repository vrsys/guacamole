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
#include <gua/physics/Point2PointConstraint.hpp>

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

Point2PointConstraint::Point2PointConstraint(RigidBodyNode* body_a, const math::vec3& pivot_a) : Constraint(body_a, nullptr), pivot_a_(pivot_a, false)
{
    auto ct = new btPoint2PointConstraint(*(body_a->get_bullet_rigid_body()), math::vec3_to_btVector3(pivot_a));
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    pivot_b_ = std::make_pair(math::btVector3_to_vec3(ct->getPivotInB()), false);
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

Point2PointConstraint::Point2PointConstraint(RigidBodyNode* body_a, RigidBodyNode* body_b, const math::vec3& pivot_a, const math::vec3& pivot_b)
    : Constraint(body_a, body_b), pivot_a_(pivot_a, false), pivot_b_(pivot_b, false)
{
    auto ct = new btPoint2PointConstraint(*(body_a->get_bullet_rigid_body()), *(body_b->get_bullet_rigid_body()), math::vec3_to_btVector3(pivot_a), math::vec3_to_btVector3(pivot_b));
    breaking_impulse_threshold_.first = ct->getBreakingImpulseThreshold();
    ct_ = ct;
}

////////////////////////////////////////////////////////////////////////////////

void Point2PointConstraint::set_pivot_a(const math::vec3& pivot)
{
    if(pivot_a_.first == pivot)
        return;
    lock_guard<SpinLock> lk(lock_);
    pivot_a_ = std::make_pair(pivot, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void Point2PointConstraint::set_pivot_b(const math::vec3& pivot)
{
    if(pivot_b_.first == pivot)
        return;
    lock_guard<SpinLock> lk(lock_);
    pivot_b_ = std::make_pair(pivot, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Point2PointConstraint::update_constraint()
{
    auto ct = static_cast<btPoint2PointConstraint*>(ct_);
    if(pivot_a_.second)
    {
        ct->setPivotA(math::vec3_to_btVector3(pivot_a_.first));
        pivot_a_.second = false;
    }
    if(body_b_ && pivot_b_.second)
    {
        ct->setPivotB(math::vec3_to_btVector3(pivot_b_.first));
        pivot_b_.second = false;
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
