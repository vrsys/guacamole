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
#include <gua/node/ClippingPlaneNode.hpp>

// guacamole headers
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////

ClippingPlaneNode::ClippingPlaneNode(std::string const& name, math::mat4 const& transform, Configuration configuration) : Node(name, transform), config(configuration) {}

/* virtual */ void ClippingPlaneNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////

math::vec3 ClippingPlaneNode::get_center() const { return math::get_translation(get_cached_world_transform()); }

////////////////////////////////////////////////////////////////////////////////

math::vec3 ClippingPlaneNode::get_normal() const { return math::vec3(scm::math::normalize(get_cached_world_transform() * math::vec4(0, 0, -1, 0))); }

////////////////////////////////////////////////////////////////////////////////

math::vec4 ClippingPlaneNode::get_component_vector() const
{
    auto normal(get_normal());
    auto center(get_center());
    auto dist(scm::math::dot(normal, -center));
    return math::vec4(normal.x, normal.y, normal.z, dist);
}

////////////////////////////////////////////////////////////////////////////////

bool ClippingPlaneNode::is_visible(int view_id) const
{
    bool visible(false);

    if(config.view_ids().empty())
    {
        visible = true;
    }
    else
    {
        visible = std::find(config.view_ids().begin(), config.view_ids().end(), view_id) != config.view_ids().end();
    }

    return visible;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> ClippingPlaneNode::copy() const { return std::make_shared<ClippingPlaneNode>(*this); }

////////////////////////////////////////////////////////////////////////////////

void ClippingPlaneNode::set_scenegraph(SceneGraph* scenegraph)
{
    if(scenegraph_)
    {
        scenegraph_->remove_clipping_plane_node(this);
    }

    Node::set_scenegraph(scenegraph);

    if(scenegraph_)
    {
        scenegraph_->add_clipping_plane_node(this);
    }
}

} // namespace node
} // namespace gua
