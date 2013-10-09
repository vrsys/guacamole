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

#ifndef GUA_SCENE_GRAPH_HPP
#define GUA_SCENE_GRAPH_HPP

#include <gua/scenegraph/Node.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/logger.hpp>

#include <memory>
#include <string>
#include <set>

namespace gua {

class NodeVisitor;
class RayNode;

/**
 * A class to represent a scene.
 *
 * This class is used to build and structure a graph describing a scene with
 * all its contents. It provides an interface to set up and have access to
 * a graph consisting of several Nodes in order to build a scene abstraction.
 */
class SceneGraph {

 public:
  class Iterator;

 public:

  /**
   * Constructor.
   *
   * This constructs an empty SceneGraph.
   */
  SceneGraph(std::string const& name = "scenegraph");

  /**
   * Copy Constructor.
   *
   * This constructs a copy of a given SceneGraph.
   *
   * \param graph    The SceneGraph to be copied.
   */
  SceneGraph(SceneGraph const& graph);

  /**
   * Adds a new Node.
   *
   * This function adds a new Node to the graph. If the given path to a
   * parent Node is invalid (this means this Node doesn't exist), an
   * Node on the SceneGraph's "end" is returned. Otherwise an
   * Node on the added Node is given back.
   *
   * \param path_to_parent The location of the Node the new Node will be
   *                       attached to.
   * \param node_name      The name of the new Node.
   * \param core           The core the new Node shall refer to.
   * \param transform      The transformation of the object the new Node
   *                       carries.
   *
   * \return Iterator      An Iterator on the recently added Node. If the
   *                       path to the parent was invalid, the returned
   *                       Iterator points to the SceneGraph's "end".
   */
  template<typename T>
  std::shared_ptr<T> add_node(std::string const& path_to_parent, std::string const& node_name) {

    auto new_node(std::make_shared<T>(node_name));

    std::shared_ptr<Node> const& parent(find_node(path_to_parent));

    if (!parent) {
      WARNING("A node with the name %s does not exist!", path_to_parent.c_str());
      return new_node;
    }

    return add_node(parent, new_node);
  }

  template<typename T>
  std::shared_ptr<T> add_node(std::shared_ptr<Node> const&  parent, std::string const& node_name) {

    auto new_node(std::make_shared<T>(node_name));

    parent->add_child(new_node);
    return new_node;
  }

  template<typename T>
  std::shared_ptr<T> add_node(std::string const& path_to_parent, std::shared_ptr<T> const& new_node) {

    std::shared_ptr<Node> const& parent(find_node(path_to_parent));

    if (!parent) {
      WARNING("A node with the name %s does not exist!", path_to_parent.c_str());
      return new_node;
    }

    return add_node(parent, new_node);
  }

  template<typename T>
  std::shared_ptr<T> add_node(std::shared_ptr<Node> const&  parent, std::shared_ptr<T> const& new_node) {
    parent->add_child(new_node);
    return new_node;
  }

  /**
   * Removes a Node.
   *
   * This function removes a Node from the graph and returnes an Iterator
   * on the next Node with respect to the Iterator's traversion style.
   *
   * \param path_to_node   The location of the Node to be removed.
   *
   * \return Iterator      An Iterator on the next Node.
   */
  void remove_node(std::string const& path_to_node);
  void remove_node(std::shared_ptr<Node> const& to_remove);

  /**
   * Returns an iterator to a Node.
   *
   * This function returns an iterator to a Node which may be used to
   * apply operations.
   *
   * \param path_to_node   The location of the Node to be encapsulated in
   *                       the Iterator.
   *
   * \return Iterator      An Iterator on the given Node.
   */
  Iterator get_iterator(std::string const& path_to_node) const;

  /**
   * Returns an iterator to the beginning of the SceneGraph.
   *
   * This function returns an iterator to the beginning of the
   * SceneGraph which is a Node named "/".
   *
   * \return Iterator      An Iterator on the Node "/".
   */
  Iterator begin() const;

  /**
   * Returns an iterator to the "end" of the SceneGraph.
   *
   * Because the SceneGraph is structured as a non-cyclic graph is
   * expected to be, there is no real "end". Therefore this function
   * returns an Iterator on an imaginary end, which can be used to
   * check whether an added Node is valid or an iteration over the graph
   * has finished.
   *
   * \return Iterator      An Iterator on the "end" of the SceneGraph.
   */
  Iterator end() const;

  void set_name(std::string const& name);
  std::string const& get_name() const;

  void set_root(std::shared_ptr<Node> const& root);
  std::shared_ptr<Node> const& get_root() const;

  /**
   * Allows to access nodes via the index operator.
   *
   * This operator allows to access nodes via the index operator. If a
   * given path doesn't refer to an existing set of nodes, all missing
   * nodes in the path are added to the SceneGraph.
   *
   * \param path_to_node   The path to the Node you want to access.
   *
   * \return Iterator      An Iterator on the given Node.
   */
  std::shared_ptr<Node> operator[](std::string const& path_to_node) const;
  SceneGraph const& operator=(SceneGraph const& rhs);

  void to_dot_file(std::string const& file) const;

  void update_cache() const;
  void accept(NodeVisitor& visitor) const;

  std::set<PickResult> const ray_test(RayNode const& ray,
                                      PickResult::Options options = PickResult::PICK_ALL,
                                      std::string const& mask = "");

 private:

  std::shared_ptr<Node> find_node(std::string const& path_to_node,
                  std::string const& path_to_start = "/") const;

  bool has_child(std::shared_ptr<Node> const& parent, std::string const& child_name) const;

  std::shared_ptr<Node> root_;
  std::string name_;
};

/**
 * The stream operator.
 *
 * This operator allows to stream the names of the SceneGraph's nodes into a
 * given ostream.
 *
 * \param os         The ostream the names will be streamed into.
 * \param graph      The SceneGraph to be streamed.
 *
 * \return ostream   A reference to the manipulated stream given to the
 *                  function.
 */
std::ostream& operator<<(std::ostream& os, SceneGraph const& graph);

}

#endif  // GUA_SCENE_GRAPH_HPP
