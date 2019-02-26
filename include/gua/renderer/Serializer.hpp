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

#ifndef GUA_SERIALIZER_HPP
#define GUA_SERIALIZER_HPP

#include <stack>

// guacamole headers
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/utils/Mask.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>

namespace gua
{
class SceneGraph;

namespace node
{
class SerializableNode;
}

/**
 * This class is used to convert the scengraph to a (opimized) sequence.
 *
 * It serializes the scene graph.
 */
class Serializer : public NodeVisitor
{
  public:
    /**
     * Constructor.
     *
     * This constructs an Serializer.
     */
    Serializer();

    /**
     * Takes the Scengraph and processes geometry, light and camera
     *        lists.
     *
     * \param scene_graph          The SceneGraph to be processed.
     * \param render_mask          The mask to be applied to the nodes of
     *                             the graph.
     */
    void check(SerializedScene& output, SceneGraph const& scene_graph, Mask const& mask, bool enable_frustum_culling, int view_id);

    /**
     * Visits a TransformNode
     *
     * This function provides the interface to visit a TransformNode
     *
     * \param cam   Pointer to TransformNode
     */
    void visit(node::Node* node) override;

    /**
     * Visits an LODNode
     *
     * This function provides the interface to visit an LODNode
     *
     * \param cam   Pointer to LODNode
     */
    void visit(node::LODNode* lod) override;

    /**
     * Visits a GeometryNode
     *
     * This function provides the interface to visit a GeometryNode
     *
     * \param geometry   Pointer to GeometryNode
     */
    void visit(node::SerializableNode* geometry) override;

  private:
    bool is_visible(node::Node* node) const;
    bool check_clipping_planes(node::Node* node) const;

    void visit_children(node::Node* node);

    Frustum culling_frustum_;
    Frustum rendering_frustum_;
    Mask render_mask_;

    SerializedScene* data_;

    bool enable_frustum_culling_;
    bool enable_alternative_frustum_culling_;
};

} // namespace gua

#endif // GUA_SERIALIZER_HPP
