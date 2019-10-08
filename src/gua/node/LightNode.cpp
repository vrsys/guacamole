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
#include <gua/node/LightNode.hpp>

// guacamole header
#include <gua/platform.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua
{
namespace node
{
LightNode::LightNode(std::string const& name, Configuration const& configuration, math::mat4 const& transform) : SerializableNode(name, transform), data(configuration) {}

void LightNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

void LightNode::update_bounding_box() const
{
    if(data.get_type() == LightNode::Type::SUN)
    {
        bounding_box_ = math::BoundingBox<math::vec3>(math::vec3(std::numeric_limits<math::vec3::value_type>::lowest()), math::vec3(std::numeric_limits<math::vec3::value_type>::max()));
        return;
    }

    math::BoundingBox<math::vec3> geometry_bbox;

    if(data.get_type() == LightNode::Type::POINT)
    {
        geometry_bbox = GeometryDatabase::instance()->lookup("gua_light_sphere_proxy")->get_bounding_box();
    }
    else
    {
        geometry_bbox = GeometryDatabase::instance()->lookup("gua_light_cone_proxy")->get_bounding_box();
    }

    bounding_box_ = transform(geometry_bbox, world_transform_);

    for(auto child : get_children())
    {
        bounding_box_.expandBy(child->get_bounding_box());
    }
}

std::shared_ptr<Node> LightNode::copy() const { return std::make_shared<LightNode>(*this); }

} // namespace node
} // namespace gua
