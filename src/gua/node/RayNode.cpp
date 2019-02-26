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
#include <gua/node/RayNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/KDTree.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua
{
namespace node
{
const float RayNode::END(std::numeric_limits<float>::max());

RayNode::RayNode(std::string const& name, math::mat4 const& transform) : Node(name, transform) {}

void RayNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

std::pair<float, float> RayNode::intersect(math::BoundingBox<math::vec3> const& box) const { return ::gua::intersect(get_world_ray(), box); }

Ray RayNode::get_world_ray() const
{
    math::mat4 world_transform(get_world_transform());

    math::vec4 origin(0, 0, 0, 1.0);
    math::vec4 direction(0, 0, -1, 0.0);

    origin = world_transform * origin;
    direction = world_transform * direction;

    return Ray(origin, direction, 1.0);
}

void RayNode::update_bounding_box() const
{
    bounding_box_ = math::BoundingBox<math::vec3>();

    for(auto child : get_children())
    {
        bounding_box_.expandBy(child->get_bounding_box());
    }
}

std::shared_ptr<Node> RayNode::copy() const { return std::make_shared<RayNode>(*this); }

} // namespace node
} // namespace gua
