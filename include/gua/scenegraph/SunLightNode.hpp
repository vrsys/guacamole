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

#ifndef GUA_SUN_LIGHT_NODE_HPP
#define GUA_SUN_LIGHT_NODE_HPP

#include <gua/platform.hpp>
#include <gua/scenegraph/Node.hpp>

#include <gua/utils/Color3f.hpp>
#include <gua/utils/configuration_macro.hpp>

#include <string>

namespace gua {

/**
 * This class is used to represent directional and parallel light in the
 * SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL SunLightNode : public Node {
 public:

  struct Configuration {
    /**
     * The color of the light source.
     * It's possible to use negative values and values > 1.
     */
    GUA_ADD_PROPERTY(utils::Color3f,      color,                                  utils::Color3f(1.f, 1.f, 1.f));

    /**
     * Triggers whether the light casts shadows.
     */
    GUA_ADD_PROPERTY(bool,                enable_shadows,                         false);

    /**
     * Triggers volumetric screen-space effects for the light source.
     */
    GUA_ADD_PROPERTY(bool,                enable_godrays,                         false);

    /**
     * Triggers whether or not the light source has influence on objects'
     * diffuse shading.
     */
    GUA_ADD_PROPERTY(bool,                enable_diffuse_shading,                 true);

    /**
     * Triggers whether or not the light source has influence on objects'
     * specular shading.
     */
    GUA_ADD_PROPERTY(bool,                enable_specular_shading,                true);

    /**
     * Sets the size in pixel of the texture used for shadow map generation.
     * Choose wisely!
     */
    GUA_ADD_PROPERTY(unsigned,            shadow_map_size,                        1024);

    /**
     * Sets the offset between a shadow casting object and the shadow's edge.
     * Increase this value to reduce artefacts (especially shadow acne).
     */
    GUA_ADD_PROPERTY(float,               shadow_offset,                          0.001f);

    /**
     * Sets the split distance-to-camera values for rendering cascaded shadow
     * maps in world coordinates.
     */
    GUA_ADD_PROPERTY(std::vector<float>,  shadow_cascaded_splits,                 std::vector<float>({0.1f, 2, 10, 50, 100.f}));

    /**
     * Sets the value used for near clipping when renering cascaded shadow maps.
     */
    GUA_ADD_PROPERTY(float,               shadow_near_clipping_in_sun_direction,  100.f);
  };

  /**
   * The SunLightNode's configuration.
   */
  Configuration data;

  /**
   * Constructor.
   *
   * This constructs an empty SunLightNode.
   *
   */
  SunLightNode() {}

  /**
   * Constructor.
   *
   * This constructs a SunLightNode with the given parameters.
   *
   * \param name           The name of the new SunLightNode.
   * \param configuration  A configuration struct to define the SunLightNode's
   *                       properties.
   * \param transform      A matrix to describe the SunLightNode's
   *                       transformation. The default light direction is -y.
   *                       NOTE: Since a SunLightNode is a directional light
   *                       source, only rotations are needed to describe its
   *                       transformation.
   */
  SunLightNode(std::string const& name,
                Configuration const& configuration = Configuration(),
                math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the SunLightNode's data.
   */
  /* virtual */ void accept(NodeVisitor&);

  /**
   * Updates a SunLightNode's BoundingBox.
   *
   * The bounding box is updated according to the transformation matrices of
   * all children.
   */
  void update_bounding_box() const;

 private:

  std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_SUN_LIGHT_NODE_HPP
