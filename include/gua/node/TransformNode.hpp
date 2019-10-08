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

#ifndef GUA_GROUP_NODE_HPP
#define GUA_GROUP_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/Node.hpp>

namespace gua
{
namespace node
{
/**
 * This class is used to represent an transformation node in the SceneGraph.
 *
 * Because any of guacamole's Nodes stores children and transformation, the
 * TransformationNode only exists for the convenience of explicitly limiting a
 * Node's purpose to the transformation of several children.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL TransformNode : public Node
{
  public:
    /**
     * Constructor.
     *
     * This constructs an empty TransformNode.
     *
     */
    TransformNode() = default;

    /**
     * Constructor.
     *
     * This constructs a TransformNode with the given parameters.
     *
     * \param name           The name of the new TransformNode.
     * \param transform      A matrix to describe the TransformNode's
     *                       transformation.
     */
    TransformNode(std::string const& name, math::mat4 const& transform = math::mat4::identity());

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the TransformNode's data.
     */
    void accept(NodeVisitor& visitor) override;

  private:
    std::shared_ptr<Node> copy() const override;
};

} // namespace node
} // namespace gua

#endif // GUA_GROUP_NODE_HPP
