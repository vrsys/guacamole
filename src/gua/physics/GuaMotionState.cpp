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
#include <gua/physics/GuaMotionState.hpp>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

GuaMotionState::GuaMotionState(const btTransform& start_trans) : dirty(false)
{
    for(int i(0); i < 3; ++i)
        transforms_[i] = new btTransform(start_trans);
}

////////////////////////////////////////////////////////////////////////////////

GuaMotionState::~GuaMotionState()
{
    for(int i(0); i < 3; ++i)
        delete transforms_[i];
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void GuaMotionState::getWorldTransform(btTransform& centerOfMassWorldTrans) const { centerOfMassWorldTrans = *transforms_[0]; }

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void GuaMotionState::setWorldTransform(const btTransform& centerOfMassWorldTrans) { *transforms_[0] = centerOfMassWorldTrans; }

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void GuaMotionState::latest_transform(btTransform& centerOfMassWorldTrans) const { centerOfMassWorldTrans = *transforms_[2]; }

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
