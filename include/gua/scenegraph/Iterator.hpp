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

#ifndef GUA_ITERATOR_HPP
#define GUA_ITERATOR_HPP

// guacamole headers
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/utils/logger.hpp>

// external headers
#include <memory>
#include <map>
#include <list>

namespace gua {

class Node;

/**
 * This class is used to iterate over the SceneGraph.
 *
 * Iterators are used to give access to the SceneGraph's data without having
 * to worry about its inner structure. They are very useful to get access to
 * the nodes one by one in breadth or depth first style traversion.
 *
 */
class SceneGraph::Iterator {
 public:

  Iterator();
  /**
   * Constructor.
   *
   * This constructs an Iterator on a given Node.
   *
   * \param node      The Node the Iterator shall contain.
   */
  Iterator(std::shared_ptr<Node> const& node);

  /**
   *
   */
  Node const& get_node() const;

  /**
   *
   */
  Node& get_node();

  /**
   * Increments the Iterator.
   *
   * Increments the Iterator by detecting the next node of the SceneGraph
   * with respect to the Iterator's type and setting the Iterator's node
   * to this one. If every node is visited, the Iterator will be set on
   * "end".
   */
  void operator++();

  /**
   * Compares two Iterators.
   *
   * This function returns true if two Iterators point on the same Node.
   *
   * \param rhs       The Iterator to be checked on equality with.
   *
   * \return result   The result of the comparison.
   */
  bool operator==(Iterator const& rhs);

  /**
   * Compares two Iterators.
   *
   * This function returns true if two Iterators do not point on the same
   * Node.
   *
   * \param rhs       The Iterator to be checked on equality with.
   *
   * \return result   The result of the comparison.
   */
  bool operator!=(Iterator const& rhs);

  operator bool() const;

  std::shared_ptr<Node> const& operator*() const;

  std::shared_ptr<Node> const& operator->() const;

 private:
  mutable std::shared_ptr<Node> current_node_;
  std::shared_ptr<Node> start_node_;

  unsigned current_depth_;

  void next();

  std::shared_ptr<Node> get_neighbour(std::shared_ptr<Node> const& to_be_checked);

  static const std::string end_name_;
  static const math::mat4 end_transform_;
};

}

#endif  // GUA_ITERATOR_HPP
