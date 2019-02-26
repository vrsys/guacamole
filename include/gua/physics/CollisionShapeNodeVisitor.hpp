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

#ifndef GUA_COLLISION_SHAPE_NODE_VISITOR_HPP
#define GUA_COLLISION_SHAPE_NODE_VISITOR_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Mask.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/math/math.hpp>

#include <gua/node/Node.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/node/GeometryNode.hpp>
#include <gua/node/LightNode.hpp>
#include <gua/node/ScreenNode.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/node/TexturedQuadNode.hpp>

// external headers
#include <stack>

namespace gua
{
namespace physics
{
/**
 * This class visits a RigidBodyNode subgraph and collects collision
 *        shapes from there.
 *
 */
class CollisionShapeNodeVisitor : public NodeVisitor
{
  public:
    /**
     * Constructor.
     *
     * This constructs an CollisionShapeNodeVisitor.
     */
    CollisionShapeNodeVisitor();

    /**
     * Synchronizes collision shapes associated with the given
     *        rigid body.
     *
     * \param rigid_body  The rigid body that needs to be traversed.
     */
    void check(RigidBodyNode* rigid_body);

    /**
     * Visits a RigidBodyNode
     *
     * This function visits a RigidBodyNode
     *
     * \param node   Pointer to RigidBodyNode
     */
    /* virtual */ void visit(RigidBodyNode* node) override;

    /**
     * Visits a CollisionShapeNode
     *
     * This function visits a CollisionShapeNode
     *
     * \param node   Pointer to CollisionShapeNode
     */
    /* virtual */ void visit(CollisionShapeNode* node) override;

    ///@{
    /**
     * Visits a Node
     *
     * This function visits a Node
     *
     * \param node    Pointer to Node
     */

    /* virtual */ void visit(node::TransformNode* node) override { generic_visit(node); }

    /* virtual */ void visit(node::GeometryNode* node) override { generic_visit(node); }

    /* virtual */ void visit(node::LightNode* node) override { generic_visit(node); }

    /* virtual */ void visit(node::ScreenNode* node) override { generic_visit(node); }

    /* virtual */ void visit(node::RayNode* node) override { generic_visit(node); }

    /* virtual */ void visit(node::TexturedQuadNode* node) override { generic_visit(node); }

    ///@}

  private:
    void generic_visit(node::Node* node);
    void push_stack(math::mat4 const& current_matrix);
    void pop_stack(node::Node* new_node);

    RigidBodyNode* rigid_body_;
    std::stack<math::mat4> matrix_stack_;
    int last_depth_;

    bool resync_;
    unsigned shape_index_;
};

} // namespace physics
} // namespace gua

#endif // GUA_COLLISION_SHAPE_NODE_VISITOR_HPP
