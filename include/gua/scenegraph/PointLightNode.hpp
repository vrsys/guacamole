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

#ifndef GUA_POINT_LIGHT_NODE_HPP
#define GUA_POINT_LIGHT_NODE_HPP

#include <gua/platform.hpp>
#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

#include <gua/utils/Color3f.hpp>

#include <string>

namespace gua {

/**
 * This class is used to represent a point light in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL PointLightNode : public Node {
 public:

  struct Configuration {
    /**
     * The color of the light source.
     * It's possible to use negative values and values > 1.
     */
    GUA_ADD_PROPERTY(utils::Color3f,  color,                    utils::Color3f(1.f, 1.f, 1.f));

    /**
     * The exponent of distance attenuation.
     * E.g. a value of 2 means quadratic falloff, 1 means linear falloff
     */
    GUA_ADD_PROPERTY(float,           falloff,                  1.f);

    /**
     * Triggers whether the light casts shadows. NOTE: Not implemented yet!
     */
    GUA_ADD_PROPERTY(bool,            enable_shadows,           false);

    /**
     * Triggers volumetric screen-space effects for the light source.
     */
    GUA_ADD_PROPERTY(bool,            enable_godrays,           false);

    /**
     * Triggers whether or not the light source has influence on objects'
     * diffuse shading.
     */
    GUA_ADD_PROPERTY(bool,            enable_diffuse_shading,   true);

    /**
     * Triggers whether or not the light source has influence on objects'
     * specular shading.
     */
    GUA_ADD_PROPERTY(bool,            enable_specular_shading,  true);

    /**
     * Sets the size in pixel of the texture used for shadow map generation.
     * Choose wisely!
     */
    GUA_ADD_PROPERTY(unsigned,        shadow_map_size,          512);
  };

  /**
   * The PointLightNode's configuration.
   */
  Configuration data;

  /**
   * Constructor.
   *
   * This constructs an empty PointLightNode.
   *
   */
  PointLightNode() {}

  /**
   * Constructor.
   *
   * This constructs a PointLightNode with the given parameters.
   *
   * \param name           The name of the new PointLightNode.
   * \param configuration  A configuration struct to define the PointLightNode's
   *                       properties.
   * \param transform      A matrix to describe the PointLightNode's
   *                       transformation.
   */
  PointLightNode(std::string const& name,
                 Configuration const& configuration = Configuration(),
                 math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the PointLightNode's data.
   */
  /* virtual */ void accept(NodeVisitor& visitor);

  /**
   * Updates a PointLightNode's BoundingBox.
   *
   * The bounding box is updated according to the transformation matrices of
   * all children.
   */
  void update_bounding_box() const;

 private:
  /**
   *
   */
  std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_POINT_LIGHT_NODE_HPP
