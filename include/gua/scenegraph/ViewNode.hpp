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

#ifndef GUA_VIEW_NODE_HPP
#define GUA_VIEW_NODE_HPP

#include <gua/scenegraph/Node.hpp>
#include <gua/utils/configuration_macro.hpp>

/**
 * This class is used to represent a camera in the SceneGraph.
 *
 */

namespace gua {

class ViewNode : public Node {
 public:

  struct Configuration {
    GUA_ADD_PROPERTY(float, stereo_width, 0.07f);
  };

  Configuration data;

  ViewNode() {}

  /**
   * Constructor.
   *
   * This constructs a ViewNode with the given parameters and calls
   * the constructor of base class Core with the type CAMERA.
   *
   * \param stereo_width  The gap between the eyes.
   */
  ViewNode(std::string const& name,
             Configuration const& configuration = Configuration(),
             math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method
   *
   * This method implements the visitor pattern for Nodes
   *
   */
  /* virtual */ void accept(NodeVisitor&);

 private:

  /**
   *
   */
  std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_VIEW_NODE_HPP
