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

#ifndef GUA_NODE_HPP
#define GUA_NODE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/math/math.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/scenegraph/PickResult.hpp>
#include <gua/utils/Mask.hpp>

// external headers
#include <map>
#include <set>
#include <list>
#include <vector>
#include <memory>

namespace gua {

/**
 * This class is used to build the internal structure of a SceneGraph.
 *
 * Nodes have a name and hold objects and there transforms. Furthermore they
 * keep track of which Nodes are attached to them (children) an to which Node
 * they are attached to (parent). Nodes can be assigned a group name which
 * allows to group them concerning similar properties etc.
 *
 * NOTE: This class is ment to be used inside the SceneGraph class only!
 *       All interaction with the graph must be handled with Iterators and via
 *       the SceneGraph interface.
 *
 */

class NodeVisitor;
class RayNode;

namespace physics { class CollisionShapeNodeVisitor; }

class GUA_DLL Node {
 public:

  /**
   * Constructor.
   *
   * This constructs a Node with the given parameters.
   *
   * \param name      The Node's name
   * \param transform The transformation of the object the Node contains.
   * \param core      The Core of the node, representing its containing
   *                 object.
   */
  Node(std::string const& name = "",
       math::mat4 const& transform = math::mat4::identity());

  /**
   * Destructor.
   *
   * This destructs a Node and all its children.
   *
   */
  virtual ~Node();

  /**
   * Accepts a visitor and calls concrete visit method
   *
   * This method implements the visitor pattern for Nodes
   *
   * \param visitor  A concrete NodeVisitor
   */
  virtual void accept(NodeVisitor& visitor) = 0;

  virtual void update_cache();

  /**
   * Returns the Node's name.
   *
   * \return string   The Node's name.
   */
  inline std::string const& get_name() const { return name_; }

  inline void set_name(std::string const& name) { name_ = name; }

  /**
   * Adds a child.
   *
   * This adds a Node to the Node's children list.
   *
   * \param child     The Node to be added as a child.
   */
  template<typename T>
  std::shared_ptr<T> add_child(std::string const& node_name) {

    auto new_node(std::make_shared<T>(node_name));
    return add_child(new_node);
  }

  template<typename T>
  std::shared_ptr<T> add_child(std::shared_ptr<T> const& new_node) {

    children_.push_back(new_node);
    new_node->parent_ = this;

    set_dirty();

    return new_node;
  }

  /**
   * Returns the Node's children list.
   *
   * \return list     The Node's children list.
   */
  inline std::vector<std::shared_ptr<Node>> const& get_children() const { return children_; }

  void clear_children();

  /**
   * Adds the Node to a group.
   *
   * \param group     The name of the group the Node will be added to.
   */
  void add_to_group(std::string const& group);

  /**
   * Adds the Node to several groups.
   *
   * \param groups    The names of the groups the Node will be added to.
   */
  void add_to_groups(std::set<std::string> const& groups);

  /**
   * Removes the Node from a group.
   *
   * \param group     The name of the group the Node will removed from.
   */
  void remove_from_group(std::string const& group);

  /**
   * Checks whether the Node is in a certain group.
   *
   * \param group        The name of the group to be checked.
   *
   * \return is_in_group Returns true if the Node is in the given group,
   *                     else false.
   */
  bool is_in_group(std::string const& group) const;

  /**
   * Gets the groups the Node is in.
   *
   * \return groups   Returns all groups the Node is in.
   */
  inline std::set<std::string> const& get_groups() const {
    return group_list_;
  }


  /**
   * Returns the transformation of the object the Node contains.
   *
   * Returns the transformation accumulated with its parents so the
   * resulting matrix represents the transformation in world
   * coordinates.
   *
   * \return transform The Object's transformation.
   */
  inline virtual math::mat4 get_transform() const { return transform_; }


  /**
   * Returns the transformation of the object the Node contains.
   *
   * Returns the transformation accumulated with its parents so the
   * resulting matrix represents the transformation in world
   * coordinates.
   *
   * \return transform The Object's transformation.
   */
  math::mat4 get_world_transform() const;

  math::vec3 get_world_position() const;

  /**
   * Sets the transformation of the object the Node contains.
   *
   * \param transform The new transformation of the Node's object.
   */
  virtual void set_transform(math::mat4 const& transform);

  /**
   * Applies a scaling on the Node's object's transformation.
   *
   * \param x         The x value of the scaling.
   * \param y         The y value of the scaling.
   * \param z         The z value of the scaling.
   */
  virtual void scale(float s);
  virtual void scale(float x, float y, float z);
  virtual void scale(math::vec3 const& s);

  /**
   * Applies a rotation on the Node's object's transformation.
   *
   * \param angle     The angle of the rotation in degrees.
   * \param x         The x factor of the rotation.
   * \param y         The y factor of the rotation.
   * \param z         The z factor of the rotation.
   */
  virtual void rotate(float angle, float x, float y, float z);
  virtual void rotate(float angle, math::vec3 const& axis);

  /**
   * Applies a translation on the Node's object's transformation.
   *
   * \param x         The x value of the translation.
   * \param y         The y value of the translation.
   * \param z         The z value of the translation.
   */
  virtual void translate(float x, float y, float z);
  virtual void translate(math::vec3 const& offset);

  /**
   * Returns the Node's depth.
   *
   * This function recursively computes the level the Node is on. Root
   * Node has a depth of 0.
   *
   * \return depth     The Node's depth.
   */
  int get_depth() const;

  /**
   * Returns if the Node has Children
   *
   * \return bool     Has the Node any children
   */
  inline bool has_children() const { return !children_.empty(); }

  /**
   * Returns the full path to the node.
   *
   * This function recursively computes the full path of the Node.
   *
   * \return path     The full path to the Node.
   */
  std::string get_path() const;

  /**
   * Returns the Node's parent.
   *
   * \return Node     The Node's parent.
   */
  inline Node* get_parent() const { return parent_; }
  std::shared_ptr<Node> get_parent_shared() const;

  virtual inline math::BoundingBox<math::vec3> const& get_bounding_box() const {
      return bounding_box_;
  }

  virtual std::set<PickResult> const ray_test(RayNode const& ray,
                                            PickResult::Options options = PickResult::PICK_ALL,
                                            std::string const& mask = "");

  void*     get_user_data(unsigned handle) const;
  unsigned  add_user_data(void* data);

  friend class SceneGraph;
  friend class GeometryLoader;
  friend class MeshLoader;
  friend class Serializer;
  friend class DotGenerator;
  friend class physics::CollisionShapeNodeVisitor;

  virtual void ray_test_impl(RayNode const& ray, PickResult::Options options,
                             Mask const& mask, std::set<PickResult>& hits);

 protected:
  /**
   *
   */
  virtual std::shared_ptr<Node> copy() const = 0;

  /**
   * Returns if the Node is Root
   *
   * \return bool     Is root node of Scenegraph
   */
  inline bool is_root() const { return parent_ == nullptr; }

  virtual void update_bounding_box() const;

 private:

  /**
   * Removes a child.
   *
   * This removes a Node from the Node's children list.
   *
   * \param child     The Node to be removed.
   */
  void remove_child(std::shared_ptr<Node> const& child);

  /**
   * Sets the Node's parent.
   *
   * \param parent    The new parent of the Node.
   */
  inline void set_parent(Node* parent) { parent_ = parent; }

  /**
   * Deep copies a Node with all its children.
   *
   * This function recursively generates new Nodes for the Node itself
   * and all of its children.
   *
   * \return node     A pointer of the recently generated Node.
   */
  std::shared_ptr<Node> deep_copy() const;

 private:
  // structure
  Node* parent_;
  std::vector<std::shared_ptr<Node>> children_;

  // internal annotations
  std::set<std::string> group_list_;
  std::vector<void*> user_data_;
  std::string name_;
  math::mat4 transform_;

 protected:
  void set_dirty() const;
  void set_parent_dirty() const;
  void set_children_dirty() const;
  mutable bool self_dirty_;
  mutable bool child_dirty_;

  // up (cached) annotations
  mutable math::BoundingBox<math::vec3> bounding_box_;

  // down (cached) annotations
  mutable math::mat4 world_transform_;
};

}

#endif  // GUA_NODE_HPP
