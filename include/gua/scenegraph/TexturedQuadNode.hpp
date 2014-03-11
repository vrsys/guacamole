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

  struct Configuration {
    /**
     * A string referring to an entry in guacamole's TextureDatabase.
     */
    GUA_ADD_PROPERTY(std::string,   texture,            "");

    /**
     * A vector containing width and height of the TexturedQuadNode.
     */
    GUA_ADD_PROPERTY(math::vec2,    size,               math::vec2(1.f, 1.f));

    /**
     * Sets wheter the given texture is a stereo texture. Set this to true, if
     * you want to display a texture rendered by a Pipeline with stereo enabled.
     */
    GUA_ADD_PROPERTY(bool,          is_stereo_texture,  false);

    /**
     * Triggers whether the given texture's x coordinates are flipped.
     */
    GUA_ADD_PROPERTY(bool,          flip_x,  false);

    /**
     * Triggers whether the given texture's y coordinates are flipped.
     */
    GUA_ADD_PROPERTY(bool,          flip_y,  false);
  };

  /**
   * The TexturedQuadNode's configuration.
   */
  Configuration data;

  /**
   * Constructor.
   *
   * This constructs an empty TexturedQuadNode.
   *
   */
  TexturedQuadNode() {}

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
                   Configuration const& configuration = Configuration(),
                   math::mat4 const& transform = math::mat4::identity());

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

 private:
  /**
   *
   */
  std::shared_ptr<Node> copy() const;

};

}

#endif  // GUA_TEXTURED_QUAD_NODE_HPP
