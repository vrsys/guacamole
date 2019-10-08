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

// class header
#include <gua/physics/CollisionShapeNodeVisitor.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/CollisionShapeDatabase.hpp>
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/physics/CollisionShapeNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

// external headers
#include <stack>

namespace gua
{
namespace physics
{
CollisionShapeNodeVisitor::CollisionShapeNodeVisitor() : rigid_body_(nullptr), last_depth_(0), resync_(false), shape_index_(0) {}

void CollisionShapeNodeVisitor::check(RigidBodyNode* rigid_body)
{
    rigid_body_ = rigid_body;
    rigid_body_->set_dirty();

    // clear matrix stack
    while(!matrix_stack_.empty())
        matrix_stack_.pop();
    matrix_stack_.push(math::mat4::identity());

    resync_ = false;
    shape_index_ = 0;

    // visit the rigid body
    rigid_body_->accept(*this);

    // remove excess elements from the shape list
    while(rigid_body_->shapes().size() > shape_index_)
    {
        rigid_body_->shapes().pop_back();
        resync_ = true;
    }

    // apply collision shapes to the rigid body
    if(resync_)
        rigid_body_->sync_shapes();
}

/* virtual */ void CollisionShapeNodeVisitor::visit(RigidBodyNode* node)
{
    if(node != rigid_body_)
    {
        generic_visit(node);
    }
    else if(node->has_children())
    {
        last_depth_ = node->get_depth();
        for(auto child : node->children_)
            child->accept(*this);
    }
}

/* virtual */ void CollisionShapeNodeVisitor::visit(CollisionShapeNode* node)
{
    pop_stack(node);
    math::mat4 curr_matrix(matrix_stack_.top() * node->get_transform());

    if(shape_index_ >= rigid_body_->shapes().size())
    {
        rigid_body_->shapes().push_back(RigidBodyNode::ShapeElement());
        resync_ = true;
    }
    RigidBodyNode::ShapeElement& sh = rigid_body_->shapes().at(shape_index_);
    if(sh.transform != curr_matrix)
    {
        sh.transform = curr_matrix;
        resync_ = true;
    }
    if(sh.shape_name != node->data.get_shape())
    {
        sh.shape = CollisionShapeDatabase::instance()->lookup(node->data.get_shape());
        sh.shape_name = node->data.get_shape();
        resync_ = true;
    }
    ++shape_index_;

    // visit children of the collision shape
    if(node->has_children())
    {
        push_stack(curr_matrix);

        for(auto child : node->children_)
            child->accept(*this);
    }
}

void CollisionShapeNodeVisitor::generic_visit(node::Node* node)
{
    pop_stack(node);
    math::mat4 curr_matrix(matrix_stack_.top() * node->get_transform());

    if(node->has_children())
    {
        push_stack(curr_matrix);

        for(auto child : node->children_)
            child->accept(*this);
    }
}

void CollisionShapeNodeVisitor::push_stack(math::mat4 const& current_matrix)
{
    matrix_stack_.push(current_matrix);
    ++last_depth_;
}

void CollisionShapeNodeVisitor::pop_stack(node::Node* new_node)
{
    int new_depth = new_node->get_depth();

    while(last_depth_ >= new_depth)
    {
        matrix_stack_.pop();
        --last_depth_;
    }
}

} // namespace physics
} // namespace gua
