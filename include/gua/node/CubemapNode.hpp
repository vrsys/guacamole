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

#ifndef GUA_CUBEMAP_NODE
#define GUA_CUBEMAP_NODE

#include <gua/platform.hpp>
#include <gua/node/SerializableNode.hpp>
#include <gua/utils/configuration_macro.hpp>

#include <string>

namespace gua {
namespace node {

/**
 * This class is used to represent an ------- node in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL CubemapNode : public SerializableNode {
 public:

  struct Configuration {
    /**
     * Sets the size in pixel of each side of the texture used for depth cube map generation.
     */
    GUA_ADD_PROPERTY(unsigned,        resolution,                  64);

    /**
     * near clipping distance for depth value generation
     */
    GUA_ADD_PROPERTY(float,           near_clip,                 0.1f);

    /**
     * far clipping distance for depth value generation
     */
    GUA_ADD_PROPERTY(float,           far_clip,                  10.f);
    /**
     * Name of the depth texture in the texture database
     */
    GUA_ADD_PROPERTY(std::string,     texture_name,       "depth_cube_texture");
  };

  /**
   * The CubemapNode's configuration.
   */
  Configuration config;

  /**
   * Constructor.
   *
   * This constructs an empty CubemapNode.
   *
   */
  CubemapNode() {};

  /**
   * Constructor.
   *
   * This constructs a CubemapNode with the given parameters.
   *
   * \param name           The name of the new CubemapNode.
   * \param configuration  A configuration struct to define the CubemapNodes's
   *                       properties.
   * \param transform      A matrix to describe the CubemapNode's
   *                       transformation.
   */
  CubemapNode(std::string const& name,
              Configuration const& configuration = Configuration(),
              math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the CubemapNode's data.
   */
  void accept(NodeVisitor& visitor) override;

  void set_texture_name(std::string const& name);
  std::string get_texture_name() const;

  float get_closest_distance() const;
  float get_distance_by_local_direction(math::vec3 const& dir) const;

 private:

  float acces_texture_data(unsigned side, math::vec2 coords) const;

  std::shared_ptr<Node> copy() const override;
  std::string texture_name_;
};

} // namespace node {
} // namespace gua {

#endif  // GUA_CUBEMAP_NODE
