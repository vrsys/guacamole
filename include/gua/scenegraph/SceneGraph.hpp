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

#include <gua/platform.hpp>
#include <gua/node/Node.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/enums.hpp>

#include <memory>
#include <string>
#include <set>

namespace gua
{
class NodeVisitor;
struct Ray;

namespace node
{
class Node;
class CameraNode;
class ClippingPlaneNode;
class RayNode;
struct SerializedCameraNode;
} // namespace node

/**
 * A class to represent a scene.
 *
 * This class is used to build and structure a graph describing a scene with
 * all its contents. It provides an interface to set up and have access to
 * a graph consisting of several Nodes in order to build a scene abstraction.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL SceneGraph
{
  public:
    /**
     * Constructor.
     *
     * This constructs a new SceneGraph.
     *
     * \param name  The new SceneGraph's name.
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
     * This function adds a new Node to the SceneGraph. If the given path to a
     * parent Node is invalid (this means this Node doesn't exist), a new Node is
     * returned but not added to any parent.
     *
     * \param path_to_parent      The location of the Node the new Node will be
     *                            attached to.
     * \param node_name           The name of the new Node.
     *
     * \tparam tparam             The type of the Node to be added.
     *
     * \return std::shared_ptr<T> A shared pointer to the recently added Node.
     */
    template <typename T>
    std::shared_ptr<T> add_node(std::string const& path_to_parent, std::string const& node_name)
    {
        auto new_node(std::make_shared<T>(node_name));

        std::shared_ptr<node::Node> const& parent(find_node(path_to_parent));

        if(!parent)
        {
            Logger::LOG_WARNING << "A node with the name " << path_to_parent << " does not exist!" << std::endl;
            return new_node;
        }

        return add_node(parent, new_node);
    }

    /**
     * Adds a new Node.
     *
     * This function adds a new Node to the SceneGraph.
     *
     * \param parent              The Node the new Node will be attached to.
     * \param node_name           The name of the new Node.
     *
     * \tparam tparam             The type of the Node to be added.
     *
     * \return std::shared_ptr<T> A shared pointer to the recently added Node.
     */
    template <typename T>
    std::shared_ptr<T> add_node(std::shared_ptr<node::Node> const& parent, std::string const& node_name)
    {
        auto new_node(std::make_shared<T>(node_name));

        parent->add_child(new_node);
        return new_node;
    }

    /**
     * Adds a new Node.
     *
     * This function adds a new Node to the SceneGraph. If the given path to a
     * parent Node is invalid (this means this Node doesn't exist), a new Node is
     * returned but not added to any parent.
     *
     * \param path_to_parent      The location of the Node the new Node will be
     *                            attached to.
     * \param new_node            The Node to be attached.
     *
     * \tparam tparam             The type of the Node to be added.
     *
     * \return std::shared_ptr<T> A shared pointer to the recently added Node.
     */
    template <typename T>
    std::shared_ptr<T> add_node(std::string const& path_to_parent, std::shared_ptr<T> const& new_node)
    {
        std::shared_ptr<node::Node> const& parent(find_node(path_to_parent));

        if(!parent)
        {
            Logger::LOG_WARNING << "A node with the name " << path_to_parent << " does not exist!" << std::endl;
            return new_node;
        }

        return add_node(parent, new_node);
    }

    /**
     * Adds a new Node.
     *
     * This function adds a new Node to the SceneGraph.
     *
     * \param parent              The Node the new Node will be attached to.
     * \param new_node            The Node to be attached.
     *
     * \tparam tparam             The type of the Node to be added.
     *
     * \return std::shared_ptr<T> A shared pointer to the recently added Node.
     */
    template <typename T>
    std::shared_ptr<T> add_node(std::shared_ptr<node::Node> const& parent, std::shared_ptr<T> const& new_node)
    {
        parent->add_child(new_node);
        return new_node;
    }

    /**
     * Removes a Node.
     *
     * This function removes a Node from the SceneGraph.
     *
     * \param path_to_node   The location of the Node to be removed.
     */
    void remove_node(std::string const& path_to_node);

    /**
     * Removes a Node.
     *
     * This function removes a Node from the SceneGraph.
     *
     * \param to_remove   The Node to be removed.
     */
    void remove_node(std::shared_ptr<node::Node> const& to_remove);

    /**
     * Sets the SceneGraph's name.
     *
     * \param name   The SceneGraph's new name.
     */
    void set_name(std::string const& name);

    /**
     * Returns the SceneGraph's name.
     *
     * \return std::string   The SceneGraph's name.
     */
    std::string const& get_name() const;

    /**
     * Sets the SceneGraph's root Node.
     *
     * \param root   The SceneGraph's new root Node.
     */
    void set_root(std::shared_ptr<node::Node> const& root);

    /**
     * Returns the SceneGraph's root Node.
     *
     * \return std::shared_ptr<Node>   The SceneGraph's root Node.
     */
    std::shared_ptr<node::Node> const& get_root() const;

    /**
     * Allows to access nodes via the index operator.
     *
     * This operator allows to access nodes via the index operator. If a
     * given path doesn't refer to an existing set of nodes, a nullptr is returned.
     *
     * \param path_to_node           The path to the wanted Node.
     *
     * \return std::shared_ptr<Node> The wanted Node.
     */
    std::shared_ptr<node::Node> operator[](std::string const& path_to_node) const;

    /**
     * Assignment operator.
     *
     * This operator deep copies a given SceneGraph, assignes the copy's values to
     * the existing graph and returns a reference to it.
     *
     * \param rhs                The SceneGraph to be copied.
     *
     * \return SceneGraph const& The SceneGraph containing the copied values.
     */
    SceneGraph const& operator=(SceneGraph const& rhs);

    /**
     * Prints the SceneGraph to a file in GraphViz' dot format.
     *
     * Serializes the graph and writes all its Nodes to a file.
     *
     * \param file  Complete path to the output file. The file doesn't need to
     *              exist. The directory structure, however, does.
     */
    void to_dot_file(std::string const& file) const;

    /**
     * Updates the cache of all SceneGraph Nodes.
     *
     * Calls Node::update_cache() on the root Node.
     */
    void update_cache() const;

    /**
     * Accepts a NodeVisitor to process all SceneGraph Nodes.
     *
     * Calls Node::accept() on the root Node.
     *
     * \param visitor The NodeVisitor to pe accepted.
     */
    void accept(NodeVisitor& visitor) const;

    std::shared_ptr<SerializedScene>
    serialize(Frustum const& rendering_frustum, Frustum const& culling_frustum, math::vec3 const& reference_camera_position, bool enable_frustum_culling, Mask const& mask, int view_id) const;

    std::shared_ptr<SerializedScene> serialize(node::SerializedCameraNode const& camera, CameraMode mode) const;

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
    std::shared_ptr<SerializedScene> serialize(node::SerializedCameraNode const& camera) const;
#endif

    /**
     * Intersects a SceneGraph with a given RayNode.
     *
     * Calls Node::ray_test() on the root Node.
     *
     * \param ray       The RayNode used to check for intersections.
     * \param options   int to configure the intersection process.
     * \param mask      A mask to restrict the intersection to certain Nodes.
     */
    std::set<PickResult> const ray_test(node::RayNode const& ray, int options = PickResult::PICK_ALL, Mask const& mask = Mask());

    /**
     * Intersects a SceneGraph with a given Ray.
     *
     * Calls Node::ray_test() on the root Node.
     *
     * \param ray       The Ray used to check for intersections.
     * \param options   int to configure the intersection process.
     * \param mask      A mask to restrict the intersection to certain Nodes.
     */
    std::set<PickResult> const ray_test(Ray const& ray, int options = PickResult::PICK_ALL, Mask const& mask = Mask());

    std::vector<node::CameraNode*> const& get_camera_nodes() const { return camera_nodes_; }

    std::vector<node::ClippingPlaneNode*> const& get_clipping_plane_nodes() const { return clipping_plane_nodes_; }

    friend class ::gua::node::Node;
    friend class ::gua::node::CameraNode;
    friend class ::gua::node::ClippingPlaneNode;

  private:
    std::shared_ptr<node::Node> find_node(std::string const& path_to_node, std::string const& path_to_start = "/") const;

    bool has_child(std::shared_ptr<node::Node> const& parent, std::string const& child_name) const;

    void add_camera_node(node::CameraNode* camera);
    void remove_camera_node(node::CameraNode* camera);

    void add_clipping_plane_node(node::ClippingPlaneNode* clipping_plane);
    void remove_clipping_plane_node(node::ClippingPlaneNode* clipping_plane);

    std::shared_ptr<node::Node> root_;
    std::string name_;

    std::vector<node::CameraNode*> camera_nodes_;
    std::vector<node::ClippingPlaneNode*> clipping_plane_nodes_;
};

} // namespace gua

#endif // GUA_SCENE_GRAPH_HPP
