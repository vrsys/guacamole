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

#ifndef GUA_CLIPPING_PLANE_NODE_HPP
#define GUA_CLIPPING_PLANE_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

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
class GUA_DLL ClippingPlaneNode : public Node
{
  public:
    struct Configuration
    {
        // if not empty, the clipping plane is only active for specified view ids
        GUA_ADD_PROPERTY(std::vector<int>, view_ids, std::vector<int>());
    };

    /**
     * The CameraNode's configuration.
     */
    Configuration config;

    /**
     * Constructor.
     *
     * This constructs an empty ClippingPlaneNode.
     *
     */
    ClippingPlaneNode(){};

    /**
     * Constructor.
     *
     * This constructs a ClippingPlaneNode with the given parameters.
     *
     * \param name           The name of the new ClippingPlaneNode.
     * \param transform      A matrix to describe the ClippingPlaneNode's
     *                       transformation.
     */
    ClippingPlaneNode(std::string const& name, math::mat4 const& transform = math::mat4::identity(), Configuration configuration = Configuration());

    math::vec3 get_center() const;
    math::vec3 get_normal() const;
    math::vec4 get_component_vector() const;

    bool is_visible(int view_id) const;

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the ClippingPlaneNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    friend class Node;

  private:
    std::shared_ptr<Node> copy() const override;

    /*virtual*/ void set_scenegraph(SceneGraph* scenegraph) override;
};

} // namespace node
} // namespace gua

#endif // GUA_CLIPPING_PLANE_NODE_HPP
