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

#ifndef GUA_GROUP_NODE_HPP
#define GUA_GROUP_NODE_HPP

#include <gua/scenegraph/Node.hpp>

/**
 * This class is used to represent an empty node in the SceneGraph.
 *
 */

namespace gua {

class GroupNode : public Node {
 public:

  GroupNode() {};

  /**
   * Constructor.
   *
   * This constructs a GroupNode with the given parameters.
   *
   * \param name       The Node's name
   * \param transform  The transformation of the object the Node contains.
   */
  GroupNode(std::string const& name,
            math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method
   *
   * This method implements the visitor pattern for Nodes
   *
   */
  /* virtual */ void accept(NodeVisitor&);

 private:

  std::shared_ptr<Node> copy() const;
};

}

#endif  // GUA_GROUP_NODE_HPP
