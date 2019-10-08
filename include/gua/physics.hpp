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

#ifndef GUA_INCLUDE_PHYSICS_HPP
#define GUA_INCLUDE_PHYSICS_HPP

// main include
#include <gua/physics/Physics.hpp>

// scenegraph nodes
#include <gua/physics/CollisionShapeNode.hpp>
#include <gua/physics/RigidBodyNode.hpp>

// collision shapes
#include <gua/physics/BoxShape.hpp>
#include <gua/physics/ConvexHullShape.hpp>
#include <gua/physics/CylinderShape.hpp>
#include <gua/physics/PlaneShape.hpp>
#include <gua/physics/SphereShape.hpp>
#include <gua/physics/TriangleMeshShape.hpp>

// constraints
#include <gua/physics/FixedConstraint.hpp>
#include <gua/physics/HingeConstraint.hpp>
#include <gua/physics/Point2PointConstraint.hpp>
#include <gua/physics/SliderConstraint.hpp>

#endif // GUA_INCLUDE_PHYSICS_HPP
