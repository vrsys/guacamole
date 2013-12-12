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

#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

/**
 * This class is used to represent a screen in the SceneGraph.
 *
 */

namespace gua {

class GUA_DLL TexturedQuadNode : public Node {
 public:

  struct Configuration {
    GUA_ADD_PROPERTY(std::string,   texture,            "");
    GUA_ADD_PROPERTY(math::vec2,    size,               math::vec2(1.f, 1.f));
    GUA_ADD_PROPERTY(bool,          is_stereo_texture,  false);
    GUA_ADD_PROPERTY(bool,          flip_x,  false);
    GUA_ADD_PROPERTY(bool,          flip_y,  false);
  };

  Configuration data;

  TexturedQuadNode() {}

  TexturedQuadNode(std::string const& name,
                   Configuration const& configuration = Configuration(),
                   math::mat4 const& transform = math::mat4::identity());

  math::mat4 get_scaled_transform() const;
  math::mat4 get_scaled_world_transform() const;

  /**
   * Accepts a visitor and calls concrete visit method
   *
   * This method implements the visitor pattern for Nodes
   *
   */
  /* virtual */ void accept(NodeVisitor&);

  /*virtual*/ void update_bounding_box() const;

 private:
  /**
   *
   */
  std::shared_ptr<Node> copy() const;

};

}

#endif  // GUA_TEXTURED_QUAD_NODE_HPP
