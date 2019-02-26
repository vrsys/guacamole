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
#include <gua/node/ScreenNode.hpp>

// guacamole header
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
namespace node
{
ScreenNode::ScreenNode(std::string const& name, Configuration const& configuration, math::mat4 const& transform) : Node(name, transform), data(configuration) {}

/* virtual */ void ScreenNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

math::mat4 ScreenNode::get_scaled_transform() const
{
    math::mat4 scale(scm::math::make_scale(data.get_size().x, data.get_size().y, gua::math::float_t(1.0)));
    return get_transform() * scale;
}

math::mat4 ScreenNode::get_scaled_world_transform() const
{
    math::mat4 scale(scm::math::make_scale(data.get_size().x, data.get_size().y, gua::math::float_t(1.0)));
    return get_world_transform() * scale;
}

std::shared_ptr<Node> ScreenNode::copy() const { return std::make_shared<ScreenNode>(*this); }

} // namespace node
} // namespace gua
