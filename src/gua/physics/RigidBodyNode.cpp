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
#include <gua/physics/RigidBodyNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/Physics.hpp>
#include <gua/physics/GuaMotionState.hpp>
#include <gua/physics/CollisionShape.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/physics/PhysicsUtils.hpp>

// external headers
#include <mutex>
using std::mutex;
using std::unique_lock;

#include <iostream>

// Locks a mutex if condition is satisfied
#define CONDITION_LOCK(cond, mtx)                                                                                                                                                                      \
    unique_lock<mutex> lk;                                                                                                                                                                             \
    if(cond)                                                                                                                                                                                           \
    {                                                                                                                                                                                                  \
        lk = unique_lock<mutex>(mtx);                                                                                                                                                                  \
    }

namespace gua
{
namespace physics
{
////////////////////////////////////////////////////////////////////////////////
/*
RigidBodyNode::RigidBodyNode()
    : node::TransformNode(),
      ph_(nullptr),
      body_(nullptr),
      motion_state_(std::make_shared<GuaMotionState>()),
      mass_(1),
      inertia_(btVector3(1, 1, 1)),
      shapes_(),
      bullet_compound_shape_(nullptr),
      bullet_empty_shape_(new btEmptyShape()),
      last_body_transform_() {
  btRigidBody::btRigidBodyConstructionInfo body_ci(
      mass_, motion_state_.get(), bullet_empty_shape_, inertia_);
  body_ci.m_friction = friction;
  body_ci.m_restitution = restitution;

  body_ = new btRigidBody(body_ci);
  body_->setActivationState(DISABLE_DEACTIVATION);
}
*/

RigidBodyNode::RigidBodyNode(const std::string& name, float mass, float friction, float restitution, const math::mat4& transform)
    : node::TransformNode(name, transform), ph_(nullptr), body_(nullptr), motion_state_(std::make_shared<GuaMotionState>(math::mat4_to_btTransform(transform))), mass_(mass),
      inertia_(btVector3(1, 1, 1)), shapes_(), bullet_compound_shape_(nullptr), bullet_empty_shape_(new btEmptyShape()), last_body_transform_()
{
    btRigidBody::btRigidBodyConstructionInfo body_ci(mass_, motion_state_.get(), bullet_empty_shape_, inertia_);
    body_ci.m_friction = friction;
    body_ci.m_restitution = restitution;

    body_ = new btRigidBody(body_ci);
    body_->setActivationState(DISABLE_DEACTIVATION);
    // body_->setUserPointer(this);
}

////////////////////////////////////////////////////////////////////////////////

RigidBodyNode::~RigidBodyNode()
{
    if(bullet_compound_shape_)
        delete bullet_compound_shape_;
    delete bullet_empty_shape_;
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void RigidBodyNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////

/* virtual */ math::mat4 RigidBodyNode::get_transform() const
{
    btTransform tr;

    if(body_->isStaticObject())
        tr = body_->getCenterOfMassTransform();
    else
        motion_state_->latest_transform(tr);

    // update bounding boxes if neccessary
    if(last_body_transform_.getOrigin() != tr.getOrigin() || last_body_transform_.getBasis().getRow(0) != tr.getBasis().getRow(0) ||
       last_body_transform_.getBasis().getRow(1) != tr.getBasis().getRow(1) || last_body_transform_.getBasis().getRow(2) != tr.getBasis().getRow(2))
    {
        set_dirty();

        last_body_transform_ = tr;
    }

    return math::btTransform_to_mat4(tr);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void RigidBodyNode::set_transform(const math::mat4& transform)
{
    // TODO: It's nice to have this function block free
    //      (along with rotate and translate functions)
    CONDITION_LOCK(ph_, ph_->lock());

    if(body_->isKinematicObject())
    {
        motion_state_->setWorldTransform(math::mat4_to_btTransform(transform));
    }
    else
    {
        body_->setWorldTransform(math::mat4_to_btTransform(transform));
        body_->activate();
    }

    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void RigidBodyNode::rotate(float angle, float x, float y, float z)
{
    CONDITION_LOCK(ph_, ph_->lock());

    if(body_->isKinematicObject())
    {
        btTransform t;
        motion_state_->getWorldTransform(t);
        t.setRotation(t.getRotation() + btQuaternion(btVector3(x, y, z), angle));
        motion_state_->setWorldTransform(t);
    }
    else
    {
        btTransform t(body_->getWorldTransform());
        t.setRotation(t.getRotation() + btQuaternion(btVector3(x, y, z), angle));
        body_->setWorldTransform(t);
        body_->activate();
    }

    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void RigidBodyNode::translate(float x, float y, float z)
{
    CONDITION_LOCK(ph_, ph_->lock());
    if(body_->isKinematicObject())
    {
        btTransform t;
        motion_state_->getWorldTransform(t);
        t.setOrigin(t.getOrigin() + btVector3(x, y, z));
        motion_state_->setWorldTransform(t);
    }
    else
    {
        btTransform t(body_->getWorldTransform());
        t.setOrigin(t.getOrigin() + btVector3(x, y, z));
        body_->setWorldTransform(t);
        body_->activate();
    }

    set_dirty();
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_kinematic(bool kinematic)
{
    CONDITION_LOCK(ph_, ph_->lock());
    if(kinematic)
    {
        body_->setCollisionFlags(body_->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
        body_->setActivationState(DISABLE_DEACTIVATION);
    }
    else
    {
        body_->setCollisionFlags(body_->getCollisionFlags() & ~btCollisionObject::CF_KINEMATIC_OBJECT);
        body_->activate(true);
    }
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_mass(float mass)
{
    bool was_static(mass_ == 0.f && mass > 0.f && has_static_shapes_);
    mass_ = mass;

    CONDITION_LOCK(ph_, ph_->lock());
    if(mass > 0.f && !was_static)
        body_->getCollisionShape()->calculateLocalInertia(mass_, inertia_);
    body_->setMassProps(mass_, inertia_);
    // Resync associated shapes if the body becomes dynamic and some of the
    // associated shapes do not support dynamic bodies.
    if(was_static)
        sync_shapes(true);
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_friction(float frict)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->setFriction(frict);
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_rolling_friction(float frict)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->setRollingFriction(frict);
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_restitution(float rest)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->setRestitution(rest);
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_damping(float lin_damping, float ang_damping)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->setDamping(lin_damping, ang_damping);
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::apply_force(const math::vec3& force, const math::vec3& rel_pos)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->applyForce(math::vec3_to_btVector3(force), math::vec3_to_btVector3(rel_pos));
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::apply_central_force(const math::vec3& force)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->applyCentralForce(math::vec3_to_btVector3(force));
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::apply_torque(const math::vec3& torque)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->applyTorque(math::vec3_to_btVector3(torque));
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::apply_torque_impulse(const math::vec3& torque)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->applyTorqueImpulse(math::vec3_to_btVector3(torque));
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::apply_impulse(const math::vec3& impulse, const math::vec3& rel_pos)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->applyImpulse(math::vec3_to_btVector3(impulse), math::vec3_to_btVector3(rel_pos));
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::apply_central_impulse(const math::vec3& impulse)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->applyCentralImpulse(math::vec3_to_btVector3(impulse));
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::clear_forces()
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->clearForces();
}

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_angular_velocity(const math::vec3& vel)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->setAngularVelocity(math::vec3_to_btVector3(vel));
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 RigidBodyNode::angular_velocity() const { return math::btVector3_to_vec3(body_->getAngularVelocity()); }

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::set_linear_velocity(const math::vec3& vel)
{
    CONDITION_LOCK(ph_, ph_->lock());
    body_->setLinearVelocity(math::vec3_to_btVector3(vel));
}

////////////////////////////////////////////////////////////////////////////////

math::vec3 RigidBodyNode::linear_velocity() const { return math::btVector3_to_vec3(body_->getLinearVelocity()); }

////////////////////////////////////////////////////////////////////////////////

void RigidBodyNode::sync_shapes(bool do_not_lock)
{
    has_static_shapes_ = false;
    if(shapes_.empty())
    {
        if(body_->getCollisionShape() != bullet_empty_shape_)
        {
            CONDITION_LOCK(ph_ && !do_not_lock, ph_->lock());
            body_->setCollisionShape(bullet_empty_shape_);
        }
    }
    else
    {
        if(body_->isStaticObject())
        { // Static rigid body shape construction

            if(shapes_.size() == 1 && shapes_[0].shape && shapes_[0].shape->is_static_shape())
            {
                // Rigid body has only one collision shape
                //
                // TODO: Implement offset transform for static rigid bodies.
                //      Now the transform of the collision shape is ignored.
                //      Do not forget to fix set_mass and set_kinematic funcs.
                auto sh = shapes_[0].shape->construct_static();
                assert(sh);
                CONDITION_LOCK(ph_ && !do_not_lock, ph_->lock());
                body_->setCollisionShape(sh);
                if(!shapes_[0].shape->has_identical_shape_constructor())
                    has_static_shapes_ = true;
            }
            else
            {
                // Rigid body has more than one collision shape or
                // it cannot be static
                auto cs = new btCompoundShape();
                for(auto sh : shapes_)
                {
                    if(sh.shape == nullptr)
                    {
                        std::cerr << "RigidBodyNode::sync_shapes 365 ERROR sh.shape == nullptr" << std::endl;
                    }
                    if(sh.shape && sh.shape->is_static_shape())
                    {
                        cs->addChildShape(math::mat4_to_btTransform(sh.transform), sh.shape->construct_static());
                        if(!sh.shape->has_identical_shape_constructor())
                            has_static_shapes_ = true;
                    }
                    else if(sh.shape && sh.shape->is_dynamic_shape())
                        sh.shape->construct_dynamic(cs, math::mat4_to_btTransform(sh.transform));
                }
                // If all the shapes support dynamic bodies, recalculate inertia
                //(in case if the body becomes dynamic)
                if(!has_static_shapes_)
                {
                    cs->calculateLocalInertia(mass_, inertia_);
                    body_->setMassProps(mass_, inertia_);
                }
                CONDITION_LOCK(ph_ && !do_not_lock, ph_->lock());
                body_->setCollisionShape(cs);
                if(bullet_compound_shape_)
                    delete bullet_compound_shape_;
                bullet_compound_shape_ = cs;
            }
        }
        else
        { // Dynamic rigid body shape construction

            auto cs = new btCompoundShape();

            for(auto sh : shapes_)
            {
                if(sh.shape == nullptr)
                {
                    std::cerr << "gua::RigidBodyNode::sync_shapes 395 ERROR sh.shape == nullptr" << std::endl;
                }
                if(sh.shape && sh.shape->is_dynamic_shape())
                {
                    sh.shape->construct_dynamic(cs, math::mat4_to_btTransform(sh.transform));
                }
            }
            cs->calculateLocalInertia(mass_, inertia_);
            body_->setMassProps(mass_, inertia_);
            CONDITION_LOCK(ph_ && !do_not_lock, ph_->lock());
            body_->setCollisionShape(cs);
            if(bullet_compound_shape_)
                delete bullet_compound_shape_;
            bullet_compound_shape_ = cs;
        }
    }
}

std::shared_ptr<node::Node> RigidBodyNode::copy() const { return std::make_shared<TransformNode>(*this); }

} // namespace physics
} // namespace gua
