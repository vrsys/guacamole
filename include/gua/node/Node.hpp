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
#include <gua/events/Signal.hpp>
#include <gua/utils/TagList.hpp>

// external headers
#include <map>
#include <set>
#include <list>
#include <vector>
#include <memory>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>

namespace gua
{
class WindowBase;
class NodeVisitor;
class SceneGraph;
class Serializer;
class DotGenerator;
struct SerializedScene;

struct Ray;

namespace physics
{
class CollisionShapeNodeVisitor;
}

namespace node
{
struct SerializedCameraNode;
class RayNode;

/**
 * This class is used as a base class to provide basic node behaviour.
 *
 * A Node stores a name and, a tansformation matrix and an axis-aligned
 * BoundingBox. Each of guacamole's Nodes may have multiple child Nodes and one
 * parent Node. Furthermore, Nodes can be assigned a group name which allows for
 * user-defined grouping concerning similar properties etc.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL Node
{
  public:
    /**
     * Constructor.
     *
     * This constructs a Node with the given parameters.
     *
     * \param name        The Node's name
     * \param transform   The Node's transformation.
     */
    Node(std::string const& name = "", math::mat4 const& transform = math::mat4::identity());

    /**
     * Destructor.
     *
     * This destructs a Node and all its children.
     *
     */
    virtual ~Node() = default;

    /**
     * Returns the Node's name.
     *
     * \return std::string  The Node's name.
     */
    inline std::string const& get_name() const { return name_; }

    /**
     * Sets the Node's name.
     *
     * \param name   The Node's new name.
     */
    inline void set_name(std::string const& name) { name_ = name; }

    /**
     * Adds a child.
     *
     * This adds a Node to the Node's children vector and returns a shared pointer
     * to the new child.
     *
     * \param node_name The name of the new Node to be added as a child.
     * \tparam T The type of the new Node to be added as a child.
     */
    template <typename T>
    std::shared_ptr<T> add_child(std::string const& node_name)
    {
        auto new_node(std::make_shared<T>(node_name));
        return add_child(new_node);
    }

    /**
     * Adds a child.
     *
     * This adds a Node to the Node's children vector and returns a shared pointer
     * to the new child.
     *
     * \param new_node The new Node to be added as a child.
     * \tparam T The type of the new Node to be added as a child.
     */
    template <typename T>
    std::shared_ptr<T> add_child(std::shared_ptr<T> const& new_node)
    {
        if(new_node->parent_ != nullptr)
        {
            new_node->parent_->remove_child(new_node);
        }

        children_.push_back(new_node);
        new_node->parent_ = this;

        set_dirty();

        new_node->set_scenegraph(scenegraph_);

        return new_node;
    }

    /**
     * Removes a child.
     *
     * This removes a Node from the Node's children vector.
     *
     * \param child  The Node to be removed.
     */
    void remove_child(std::shared_ptr<Node> const& child);

    /**
     * Returns the Node's children vector.
     *
     * \return std::vector<std::shared_ptr<Node>>  The Node's children vector.
     */
    inline std::vector<std::shared_ptr<Node>> const& get_children() const { return children_; }

    /**
     * Clears the Node's children vector. Additionally, the parent node pointer of
     * each child is set to nullptr.
     */
    void clear_children();

    gua::utils::TagList const& get_tags() const;
    gua::utils::TagList& get_tags();

    /**
     * Returns the Node's transformation.
     *
     * \return math::mat4  The Object's transformation.
     */
    inline virtual math::mat4 get_transform() const { return transform_; }

    /**
     * Sets the Node's transformation.
     *
     * \param transform The Node's new transformation.
     */
    virtual void set_transform(math::mat4 const& transform);

    /**
     * Returns the Node's world transformation.
     *
     * Returns the Node's transformation accumulated with its parent's
     * transformation
     *
     * \return math::mat4  The Node's world transformation.
     */
    virtual math::mat4 get_world_transform() const;

    math::mat4 const& get_cached_world_transform() const;

	math::mat4 get_latest_cached_world_transform(const WindowBase* w) const;

  private:
    virtual math::mat4 get_latest_world_transform(const WindowBase* w) const;

  public:

    events::Signal<math::mat4 const&> on_world_transform_changed;

    /**
     * Sets the Node's world transformation.
     *
     * \param transform The Node's new world transformation.
     */
    virtual void set_world_transform(math::mat4 const& transform);

    /**
     * Returns the Node's world postion.
     *
     * Accumulates the Node's transformation with its parent's transformation
     * and returns the translational part of the resulting matrix.
     *
     * \return math::vec3  The Node's world position.
     */
    math::vec3 get_world_position() const;

    /**
     * Applies a scaling on the Node's transformation.
     *
     * \param x         The x value of the scaling.
     * \param y         The y value of the scaling.
     * \param z         The z value of the scaling.
     */
    virtual void scale(math::float_t x, math::float_t y, math::float_t z);

    /**
     * Applies a scaling on the Node's transformation.
     *
     * \param s         The scaling vector to be used.
     */
    virtual void scale(math::vec3 const& s);

    /**
     * Applies a uniform scaling on the Node's transformation.
     *
     * \param s         The scaling value for all axes.
     */
    virtual void scale(math::float_t s);

    /**
     * Applies a rotation on the Node's transformation.
     *
     * \param angle     The angle of the rotation in degrees.
     * \param x         The x factor of the rotation.
     * \param y         The y factor of the rotation.
     * \param z         The z factor of the rotation.
     */
    virtual void rotate(math::float_t angle, math::float_t x, math::float_t y, math::float_t z);

    /**
     * Applies a rotation on the Node's transformation.
     *
     * \param angle     The angle of the rotation in degrees.
     * \param axis      A vector containing the rotation's axis values.
     */
    virtual void rotate(math::float_t angle, math::vec3 const& axis);

    /**
     * Applies a translation on the Node's transformation.
     *
     * \param x         The x value of the translation.
     * \param y         The y value of the translation.
     * \param z         The z value of the translation.
     */
    virtual void translate(math::float_t x, math::float_t y, math::float_t z);

    /**
     * Applies a translation on the Node's transformation.
     *
     * \param offset    The translation vector.
     */
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
     * Returns if the Node has children
     *
     * \return bool  The return value is true if the Node has any children, else
     *               false.
     */
    inline bool has_children() const { return !children_.empty(); }

    /**
     * Returns the full path to the node.
     *
     * This function recursively computes the full path of the Node. A Node's path
     * contains the names of all of its parents, concatenated by "/".
     *
     * \return std::string  The full path to the Node.
     */
    std::string get_path() const;

    /**
     * Returns a raw pointer to the Node's parent.
     *
     * \return Node*  The Node's parent.
     */
    inline Node* get_parent() const { return parent_; }

    /**
     * Returns a shared pointer to the Node's parent.
     *
     * \return std::shared_ptr<Node>  The Node's parent.
     */
    std::shared_ptr<Node> get_parent_shared() const;

    /**
     * Returns the Node's BoundingBox.
     *
     * \return math::BoundingBox<math::vec3>  The Node's BoundingBox.
     */
    virtual inline math::BoundingBox<math::vec3> const& get_bounding_box() const { return bounding_box_; }

    /**
     * Updates a Node's BoundingBox.
     *
     * The bounding box is updated according to the transformation matrices of
     * all children.
     */
    virtual void update_bounding_box() const;

    /**
     * Draw a Node's BoundingBox.
     *
     * If set to true and the current pipeline contains a bounding box pass the
     * node's bounding box will be drawn.
     */
    void set_draw_bounding_box(bool draw);
    bool get_draw_bounding_box() const;

    /**
     * Intersects a Node with a given RayNode.
     *
     * The function checks wheter a given RayNode intersects the Node or not. If
     * an intersection was found, a std::set<PickResult> is returned, containing
     * information about individual hits. The user may specify int
     * and a mask (referring to Nodes' group names) to configure the intersection
     * process.
     *
     * \param ray       The RayNode used to check for intersections.
     * \param options   int to configure the intersection process.
     * \param mask      A mask to restrict the intersection to certain Nodes.
     */
    virtual std::set<PickResult> const ray_test(RayNode const& ray, int options = PickResult::PICK_ALL, Mask const& mask = Mask());

    /**
     * Intersects a Node with a given Ray.
     *
     * The function checks wheter a given Ray intersects the Node or not. If
     * an intersection was found, a std::set<PickResult> is returned, containing
     * information about individual hits. The user may specify int
     * and a mask (referring to Nodes' group names) to configure the intersection
     * process.
     *
     * \param ray       The Ray used to check for intersections.
     * \param options   int to configure the intersection process.
     * \param mask      A mask to restrict the intersection to certain Nodes.
     */
    virtual std::set<PickResult> const ray_test(Ray const& ray, int options = PickResult::PICK_ALL, Mask const& mask = Mask());

    /**
     * Accepts a visitor and calls concrete visit method
     *
     * This method must be implemented by derived classes.
     *
     * \param visitor  A visitor to process the Node's data
     */
    virtual void accept(NodeVisitor& visitor) = 0;

    /**
     * Updates the Node's cache.
     *
     * For optimization purposes, a Node caches data each time the rendering is
     * being processed. This function computes and stores the Node's current
     * BoundingBox and world transformation.
     */
    virtual void update_cache();

    /**
     * Adds user-defined data to a Node.
     *
     * A user may store any data within one of guacamole's Nodes. This function
     * returns a handle which can be used to regain the information later.
     *
     * \param data        A pointer to the data to be stored.
     *
     * \return unsigned   A handle for later access.
     */
    unsigned add_user_data(void* data);

    /**
     * Returns user-defined data for a given handle.
     *
     *
     * \param handle  The handle belonging to the wanted data.
     *
     * \return void*  The user-defined data. This defaults to nullptr if the given
     *                handle is invalid.
     */
    void* get_user_data(unsigned handle) const;

    /**
     * \return size_t unique address of node
     */
    inline std::size_t const uuid() const { return uuid_; }

    friend class ::gua::SceneGraph;
    friend class ::gua::Serializer;
    friend class ::gua::DotGenerator;
    friend class ::gua::physics::CollisionShapeNodeVisitor;

    virtual void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits);

    /**
     *
     */
    virtual std::shared_ptr<Node> copy() const = 0;

    /**
     * Deep copies a Node with all its children.
     *
     * This function recursively generates new Nodes for the Node itself
     * and all of its children.
     *
     * \return node     A pointer of the recently generated Node.
     */
    virtual std::shared_ptr<Node> deep_copy() const;

    SceneGraph* get_scenegraph() const { return scenegraph_; }

  protected:
    /**
     * Returns if the Node is Root
     *
     * \return bool     Is root node of SceneGraph
     */
    inline bool is_root() const { return parent_ == nullptr; }

  private:
    /**
     * Sets the Node's parent.
     *
     * \param parent    The new parent of the Node.
     */
    inline void set_parent(Node* parent) { parent_ = parent; }

  private:
    // structure
    Node* parent_ = nullptr;
    std::vector<std::shared_ptr<Node>> children_;

    // internal annotations
    gua::utils::TagList tags_;
    std::vector<void*> user_data_;
    std::string name_;

  protected:
    bool is_visible_in(SerializedScene const& scene, node::SerializedCameraNode const& camera) const;

    void set_dirty() const;
    void set_parent_dirty() const;
    void set_children_dirty() const;

    virtual void set_scenegraph(SceneGraph* scenegraph);

    mutable bool self_dirty_ = true;
    mutable bool child_dirty_ = true;

    // up (cached) annotations
    mutable math::BoundingBox<math::vec3> bounding_box_;
    bool draw_bounding_box_ = false;

    // down (cached) annotations
    math::mat4 transform_ = math::mat4::identity(); // invertible affine transformation
    mutable math::mat4 world_transform_ = math::mat4::identity();

    SceneGraph* scenegraph_ = nullptr;
    std::size_t uuid_ = boost::hash<boost::uuids::uuid>()(boost::uuids::random_generator()());
};

} // namespace node
} // namespace gua

#endif // GUA_NODE_HPP