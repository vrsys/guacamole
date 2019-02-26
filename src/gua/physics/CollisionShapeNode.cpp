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
#include <gua/physics/CollisionShapeNode.hpp>

// guacamole headers
#include <gua/node/TransformNode.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////

CollisionShapeNode::CollisionShapeNode(const std::string& name, const math::mat4& transform) : node::TransformNode(name, transform) {}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void CollisionShapeNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<node::Node> CollisionShapeNode::copy() const { return std::make_shared<node::TransformNode>(*this); }

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
