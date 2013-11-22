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

/**
 * This class is used to represent light in the SceneGraph.
 *
 */

namespace gua {

class GUA_DLL PointLightNode : public Node {
 public:

  struct Configuration {
    GUA_ADD_PROPERTY(utils::Color3f,  color,                    utils::Color3f(1.f, 1.f, 1.f));
    GUA_ADD_PROPERTY(float,           falloff,                  1.f);
    GUA_ADD_PROPERTY(bool,            enable_shadows,           false);
    GUA_ADD_PROPERTY(bool,            enable_godrays,           false);
    GUA_ADD_PROPERTY(bool,            enable_diffuse_shading,   true);
    GUA_ADD_PROPERTY(bool,            enable_specular_shading,  true);
    GUA_ADD_PROPERTY(unsigned,        shadow_map_size,          512);
  };

  Configuration data;

  /**
   * Constructor.
   *
   * This constructs a PointLightNode with the given parameters and calls
   * the constructor of base class Core with the type LIGHT.
   *
   * \param color   The light's color.
   */

  PointLightNode() {}

  PointLightNode(std::string const& name,
                 Configuration const& configuration = Configuration(),
                 math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method
   *
   * This method implements the visitor pattern for Nodes
   *
   */
  /* virtual */ void accept(NodeVisitor&);

  void update_bounding_box() const;

 private:
  /**
   *
   */
  std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_POINT_LIGHT_NODE_HPP
