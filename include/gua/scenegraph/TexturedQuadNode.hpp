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

#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

namespace gua {

/**
 * This class is used to represent a textured quad in the SceneGraph.
 *
 * A TexturedQuadNode is capable of displaying any texture and even Pipeline
 * output. It may therefore be used to implement portals, mirrors etc.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL TexturedQuadNode : public Node {

 public:
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
  TexturedQuadNode(std::string const& name,
                   std::string const& texture = "",
                   math::mat4 const& transform = math::mat4::identity(),
                   math::vec2 const& size = math::vec2(1.0f, 1.0f),
                   bool is_stereo = false,
                   bool flip_x = false,
                   bool flip_y = false);

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
  /* virtual */ void accept(NodeVisitor& visitor);

  /*virtual*/ void update_bounding_box() const;

  /*virtual*/ void update_cache();

 public:  // get and set methods

  std::string const& get_texture() const;
  void set_texture(std::string const& name);

  math::vec2 const& get_size() const;
  void get_size(math::vec2 const& size);

  bool is_stereo_texture() const;
  void is_stereo_texture(bool enable);

  bool flip_x() const;
  void flip_x(bool enable);

  bool flip_y() const;
  void flip_y(bool enable);

 private:  // methods

  std::shared_ptr<Node> copy() const;

 private:  // attributes

  std::string texture_;
  math::vec2 size_;

  bool is_stereo_texture_;
  bool flip_x_;
  bool flip_y_;

};

}

#endif  // GUA_TEXTURED_QUAD_NODE_HPP
