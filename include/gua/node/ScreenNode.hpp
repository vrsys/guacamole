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

#ifndef GUA_SCREEN_NODE_HPP
#define GUA_SCREEN_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

namespace gua
{
namespace node
{
/**
 * This class is used to represent a screen in the SceneGraph.
 *
 * A ScreenNode is used to specify arbitrary viewing frustra. For that purpose,
 * additionally any of guacamole's Nodes may serve as eye representation. To
 * combine ScreenNode and eye, a Camera has to be specified.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL ScreenNode : public Node
{
  public:
    struct Configuration
    {
        /**
         * A vector containing width and height of the ScreenNode.
         */
        GUA_ADD_PROPERTY(math::vec2, size, math::vec2(1.0, 1.0));
    };

    /**
     * The ScreenNode's configuration.
     */
    Configuration data;

    /**
     * Constructor.
     *
     * This constructs an empty ScreenNode.
     *
     */

    /**
     * Constructor.
     *
     * This constructs a ScreenNode with the given parameters.
     *
     * \param name           The name of the new ScreenNode.
     * \param configuration  A configuration struct to define the ScreenNode's
     *                       properties.
     * \param transform      A matrix to describe the ScreenNode's
     *                       transformation. By default, the ScreenNode is aligned
     *                       with the xy-plane and facing in +z direction.
     */
    ScreenNode(std::string const& name, Configuration const& configuration = Configuration(), math::mat4 const& transform = math::mat4::identity());

    /**
     * Returns the ScreenNode's transformation, considering the scaling specified
     * in the Configuration.
     *
     * \return math::mat4  The ScreenNode's scaled transformation.
     */
    math::mat4 get_scaled_transform() const;

    /**
     * Returns the ScreenNode's world transformation, considering the scaling
     * specified in the Configuration.
     *
     * \return math::mat4  The ScreenNode's scaled world transformation.
     */
    math::mat4 get_scaled_world_transform() const;

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the ScreenNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    inline virtual std::string get_type_string() const {return "<ScreenNode>";}

  private:
    std::shared_ptr<Node> copy() const override;
};

} // namespace node
} // namespace gua

#endif // GUA_SCREEN_NODE_HPP
