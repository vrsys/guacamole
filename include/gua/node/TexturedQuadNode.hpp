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

#ifndef GUA_TEXTURED_QUAD_NODE_HPP
#define GUA_TEXTURED_QUAD_NODE_HPP

#include <string>

#include <gua/node/SerializableNode.hpp>
#include <gua/utils/configuration_macro.hpp>

namespace gua
{
namespace node
{
/**
 * This class is used to represent a textured quad in the SceneGraph.
 *
 * A TexturedQuadNode is capable of displaying any texture and even Pipeline
 * output. It may therefore be used to implement portals, mirrors etc.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL TexturedQuadNode : public SerializableNode
{
  public:
    struct Configuration
    {
        GUA_ADD_PROPERTY(std::string, texture, "gua_default_texture");
        GUA_ADD_PROPERTY(math::vec2, size, math::vec2(1.0, 1.0));
        GUA_ADD_PROPERTY(bool, flip_x, false);
        GUA_ADD_PROPERTY(bool, flip_y, false);
    };

    Configuration data;

    /**
     * Constructor.
     *
     * This constructs an empty TexturedQuadNode.
     *
     */
    TexturedQuadNode();

    /**
     * Constructor.
     *
     * This constructs a TexturedQuadNode with the given parameters.
     *
     * \param name           The name of the new TexturedQuadNode.
     * \param configuration  A configuration struct to define the
     *                       TexturedQuadNode's properties.
     * \param transform      A matrix to describe the TexturedQuadNode's
     *                       transformation. By default, the TexturedQuadNode is
     *                       aligned with the xy-plane and facing in +z direction.
     */
    TexturedQuadNode(std::string const& name, Configuration const& configuration = Configuration(), math::mat4 const& transform = math::mat4::identity());

    /**
     * Returns the TexturedQuadNode's transformation, considering the scaling
     * specified in the Configuration.
     *
     * \return math::mat4  The TexturedQuadNode's scaled transformation.
     */
    math::mat4 get_scaled_transform() const;

    /**
     * Returns the TexturedQuadNode's world transformation, considering the
     * scaling specified in the Configuration.
     *
     * \return math::mat4  The TexturedQuadNode's scaled world transformation.
     */
    math::mat4 get_scaled_world_transform() const;

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the TexturedQuadNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    // virtual void serialize(SerializedScene& scene, node::SerializedCameraNode const& camera) override;

    void update_bounding_box() const override;

    void update_cache() override;

    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

  private: // methods
    std::shared_ptr<Node> copy() const override;
};

} // namespace node
} // namespace gua

#endif // GUA_TEXTURED_QUAD_NODE_HPP
