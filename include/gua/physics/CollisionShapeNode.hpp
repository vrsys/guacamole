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

#ifndef GUA_COLLISION_SHAPE_NODE_HPP
#define GUA_COLLISION_SHAPE_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/utils/configuration_macro.hpp>

/**
 * This class represents collision shapes in the SceneGraph.
 *
 */

namespace gua
{
namespace physics
{
class GUA_DLL CollisionShapeNode : public node::TransformNode
{
  public:
    struct Configuration
    {
        GUA_ADD_PROPERTY(std::string, shape, "");
    };

    Configuration data;

    /**
     * Constructor.
     *
     * This constructs a CollisionShapeNode with the given parameters.
     *
     * \param name       The Node's name
     * \param transform  The transformation of the object the Node contains.
     */
    CollisionShapeNode(const std::string& name, const math::mat4& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method
     *
     * This method implements the visitor pattern for Nodes
     *
     */
    /* virtual */ void accept(NodeVisitor&);

  private:
    std::shared_ptr<node::Node> copy() const;
};

} // namespace physics
} // namespace gua

#endif // GUA_COLLISION_SHAPE_NODE_HPP
