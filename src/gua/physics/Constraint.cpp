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
#include <gua/physics/Constraint.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/Physics.hpp>
#include <gua/physics/RigidBodyNode.hpp>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

Constraint::Constraint(RigidBodyNode* body_a, RigidBodyNode* body_b) : ph_(nullptr), ct_(nullptr), body_a_(body_a), body_b_(body_b), breaking_impulse_threshold_(0.f, false) { invalid_.store(false); }

////////////////////////////////////////////////////////////////////////////////

Constraint::~Constraint()
{
    if(ph_)
        ph_->remove_constraint(this);
    if(ct_)
        delete ct_;
}

////////////////////////////////////////////////////////////////////////////////

void Constraint::set_enabled(bool enabled)
{
    std::lock_guard<SpinLock> lk(lock_);
    enabled_ = std::make_pair(enabled, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void Constraint::set_breaking_impulse_threshold(float threshold)
{
    if(breaking_impulse_threshold_.first == threshold)
        return;
    std::lock_guard<SpinLock> lk(lock_);
    breaking_impulse_threshold_ = std::make_pair(threshold, true);
    invalid_.store(true);
}

////////////////////////////////////////////////////////////////////////////////

void Constraint::validate()
{
    bool expected_val = true;
    if(invalid_.compare_exchange_weak(expected_val, false))
    {
        std::lock_guard<SpinLock> lk(lock_);
        if(breaking_impulse_threshold_.second)
        {
            ct_->setBreakingImpulseThreshold(breaking_impulse_threshold_.first);
            breaking_impulse_threshold_.second = false;
        }
        if(enabled_.second)
        {
            ct_->setEnabled(enabled_.first);
            enabled_.second = false;
        }
        update_constraint();
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
