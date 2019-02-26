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

#ifndef GUA_GEOMETRY_NODE_HPP
#define GUA_GEOMETRY_NODE_HPP

// guacamole headers
#include <gua/node/SerializableNode.hpp>
#include <gua/utils/configuration_macro.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/renderer/Material.hpp>

// external headers
#include <string>

namespace gua
{
namespace node
{
/**
 * This class is used to represent any kind of geometry in the SceneGraph.
 *
 * A GeometryNode only stores references to existing rendering assets stored in
 * guacamole's databases. GeometryNodes typically aren't instantiated directly
 * but by utilizing guacamole's GeometryLoader.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL GeometryNode : public SerializableNode
{
  public:
    /**
     * Constructor.
     *
     * This constructs an empty GeometryNode.
     *
     */
    GeometryNode(){};

    /**
     * Constructor.
     *
     * This constructs a GeometryNode with the given parameters.
     *
     * \param name           The name of the new GeometryNode.
     * \param configuration  A configuration struct to define the GeometryNode's
     *                       properties.
     * \param transform      A matrix to describe the GeometryNode's
     *                       transformation.
     */
    GeometryNode(std::string const& name, math::mat4 const& transform = math::mat4::identity(), ShadowMode shadow_mode = ShadowMode::LOW_QUALITY);

    /**
     * A value describing the shadow's quality.
     */
    ShadowMode get_shadow_mode() const { return shadow_mode_; }
    void set_shadow_mode(ShadowMode v) { shadow_mode_ = v; }

    inline void update_cache() override { Node::update_cache(); }

  protected:
    // virtual std::shared_ptr<Node> copy() const = 0;

    ShadowMode shadow_mode_;
};

} // namespace node
} // namespace gua

#endif // GUA_GEOMETRY_NODE_HPP
