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

#ifndef GUA_RIGID_BODY_NODE_HPP
#define GUA_RIGID_BODY_NODE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/node/TransformNode.hpp>

// external headers
#include <memory>
#include <vector>
#include <btBulletDynamicsCommon.h>

namespace gua
{
namespace physics
{
class Physics;
class GuaMotionState;
class CollisionShape;

/**
 * This class represents a physically simulated rigid body
 *        in the scene graph.
 *
 *
 */
class GUA_DLL RigidBodyNode : public node::TransformNode
{
    friend class Physics;
    friend class CollisionShapeNodeVisitor;

  public:
    enum class CollisionFilterGroups
    {
        DEFAULT_FILTER = btBroadphaseProxy::DefaultFilter,
        STATIC_FILTER = btBroadphaseProxy::StaticFilter,
        KINEMATIC_FILTER = btBroadphaseProxy::KinematicFilter,
        DEBRIS_FILTER = btBroadphaseProxy::DebrisFilter,
        SENSOR_TRIGGER = btBroadphaseProxy::SensorTrigger,
        CHARACTER_FILTER = btBroadphaseProxy::CharacterFilter,
        ALL_FILTER = btBroadphaseProxy::AllFilter
    };

    /**
     * Constructor.
     *
     * This constructs a RigidBodyNode with the given parameters.
     *
     * \param name        The Node's name
     * \param mass        The mass of the rigid body (static and kinematic
     *                    objects have zero mass).
     * \param friction    The friction of the rigid body.
     * \param restitution The restitution of the rigid body.
     * \param transform   The transformation of the object the Node contains.
     */
    RigidBodyNode(const std::string& name, float mass = 0, float friction = 0, float restitution = 0, const math::mat4& transform = math::mat4::identity());

    /**
     * Destructor.
     *
     * This destructs a RigidBodyNode.
     */
    virtual ~RigidBodyNode();

    /**
     * Accepts a visitor and calls concrete visit method
     *
     * This method implements the visitor pattern for Nodes
     *
     */
    /* virtual */ void accept(NodeVisitor&);

    virtual math::mat4 get_transform() const;

    virtual void set_transform(const math::mat4& transform);

    /**
     * Scaling of rigid bodies is not allowed.
     */
    virtual void scale(float x, float y, float z) {}

    virtual void rotate(float angle, float x, float y, float z);

    virtual void translate(float x, float y, float z);

    /**
     * Makes the rigid body kinematic.
     *
     * Movements of kinematic rigid bodies are controlled by the user,
     * physical simulation will not move such objects.
     * Kinematic bodies can influence other dynamic rigid bodies, but there is
     * no collision detection with static bodies.
     *
     * When the body becomes kinematic it can be moved by the user using
     * set_transform(), rotate(), and translate() methods.
     *
     * \param enabled If true, the body is kinematic; otherwise either static
     *                or dynamic.
     */
    void set_kinematic(bool kinematic);

    /**
     * Checks if the rigid body is kinematic.
     *
     * \return True if the rigid body is kinematic.
     */
    bool is_kinematic() const { return body_->isKinematicObject(); }

    /**
     * Sets the new mass of the body.
     *
     * Setting the mass to zero makes the rigid body static.
     *
     * \param mass New mass.
     */
    void set_mass(float mass);

    /**
     * Gets current mass of the rigid body.
     *
     * The mass of zero denotes non-dynamic rigid bodies.
     *
     * \return Mass value.
     */
    float mass() const { return mass_; }

    /**
     * Sets the friction of the rigid body.
     *
     * Friction applies only when rigid bodies are in contact.
     *
     * A value of zero means that the body has no friction at all.
     *
     * \param frict New friction.
     * \sa    set_rolling_friction()
     */
    void set_friction(float frict);

    /**
     * Gets current friction of the rigid body.
     *
     * \return Friction factor.
     */
    float friction() const { return body_->getFriction(); }

    /**
     * Sets the rolling friction of the rigid body.
     *
     * Rolling friction hampers in rolling of curved bodies (even on sloped
     * surfaces). Please note that the rolling friction should be set on all
     * interacting bodies to come into effect.
     *
     * \param frict New rolling friction.
     * \sa    set_friction()
     */
    void set_rolling_friction(float frict);

    /**
     * Gets current rolling friction of the rigid body.
     *
     * \return Rolling friction factor.
     */
    float rolling_friction() const { return body_->getRollingFriction(); }

    /**
     * Sets the restitution of the rigid body.
     *
     * Restitution indicates bounciness of the body. It shows how much
     * velocity is lost after collision. With a value of zero all velocity is
     * lost preventing any bouncing.
     *
     * \param rest New restitution of the body.
     */
    void set_restitution(float rest);

    /**
     * Gets current restitution of the rigid body.
     *
     * \return Restitution factor.
     */
    float restitution() const { return body_->getRestitution(); }

    /**
     * Sets the linear and angular damping coefficients.
     *
     * Damping expresses velocity attenuation. It indicates how velocity
     * (linear and angular) will be decreasing in time. In contrast to
     * friction, damping does not require bodies being in contact.
     *
     * \param lin_damping Linear damping of the body.
     * \param ang_damping Angular damping of the body.
     * \sa    set_friction()
     */
    void set_damping(float lin_damping, float ang_damping);

    /**
     * Gets current linear velocity damping.
     *
     * \return Damping factor.
     */
    float linear_damping() const { return body_->getLinearDamping(); }

    /**
     * Gets current angular velocity damping.
     *
     * \return Damping factor.
     */
    float angular_damping() const { return body_->getAngularDamping(); }

    /**
     * Applies the force to the rigid body at a given position.
     *
     * If the position is not at the center of the body, torque is
     * also applied.
     *
     * \param torque  The vector containing the force values.
     * \param rel_pos The vector containing the relative point of
     *                force application.
     * \sa    apply_central_force(), apply_torque()
     */
    void apply_force(const math::vec3& force, const math::vec3& rel_pos);

    /**
     * Applies the force at the rigid body center.
     *
     * \param force The vector containing the force valus.
     * \sa    apply_force()
     */
    void apply_central_force(const math::vec3& force);

    /**
     * Applies the torque to the rigid body.
     *
     * \param torque The vector containing the torque values.
     */
    void apply_torque(const math::vec3& torque);

    /**
     * Applies the torque impulse to the rigid body.
     *
     * \param torque The vector containing the torque impulse values.
     * \sa    apply_impulse()
     */
    void apply_torque_impulse(const math::vec3& torque);

    /**
     * Applies an impulse to the rigid body at a given position.
     *
     * If the position is not at the center of the body, torque impulse is
     * also applied.
     *
     * \param impulse  The vector containing the impulse values.
     * \param rel_pos  The vector containing the relative point of
     *                 impulse application.
     * \sa    apply_central_impulse(), apply_torque_impulse()
     */
    void apply_impulse(const math::vec3& impulse, const math::vec3& rel_pos);

    /**
     * Applies an impulse at the rigid body center.
     *
     * \param torque The vector containing the impulse values.
     * \sa    apply_impulse()
     */
    void apply_central_impulse(const math::vec3& impulse);

    /**
     * Clears all forces and torques.
     */
    void clear_forces();

    /**
     * Sets angular velocity of the rigid body.
     *
     * \param vel The vector containing new velocity values.
     * \sa    set_linear_velocity()
     */
    void set_angular_velocity(const math::vec3& vel = math::vec3());

    /**
     * Gets current angular velocity of the rigid body.
     *
     * \return Current velocity.
     * \sa     linear_velocity()
     */
    math::vec3 angular_velocity() const;

    /**
     * Sets linear velocity of the rigid body.
     *
     * \param vel The vector containing new velocity values.
     * \sa    set_angular_velocity()
     */
    void set_linear_velocity(const math::vec3& vel = math::vec3());

    /**
     * Gets current linear velocity of the rigid body.
     *
     * \return Current velocity.
     * \sa     angular_velocity()
     */
    math::vec3 linear_velocity() const;

    ///@{
    /**
     * Gets the pointer to the internal representation of the
     *        rigid body.
     *
     * \return The pointer to a Bullet's rigid body.
     */
    const btRigidBody* get_bullet_rigid_body() const { return body_; }
    btRigidBody* get_bullet_rigid_body() { return body_; }
    ///@}

    // No copying construction. No assignment.
    RigidBodyNode() = delete;
    RigidBodyNode(const RigidBodyNode&) = delete;
    RigidBodyNode(RigidBodyNode&&) = delete;
    RigidBodyNode& operator=(const RigidBodyNode&) = delete;
    RigidBodyNode& operator=(RigidBodyNode&&) = delete;

  private:
    struct ShapeElement
    {
        ShapeElement() : transform(), shape_name(""), shape(nullptr) {}
        math::mat4 transform;
        std::string shape_name;
        std::shared_ptr<CollisionShape> shape;
    };

    std::vector<ShapeElement>& shapes() { return shapes_; }

    // apply scene graph's collection of nested shapes to Bullet's
    // collision shape.
    void sync_shapes(bool do_not_lock = false);

    std::shared_ptr<node::Node> copy() const override;

    // Indicates if the body includes shapes that support static objects only.
    // Useful to check if we need to reconstruct Bullet's collision shape when
    // the type of the body changes.
    bool has_static_shapes_;

    Physics* ph_ = nullptr;
    btRigidBody* body_ = nullptr;
    std::shared_ptr<GuaMotionState> motion_state_ = nullptr;
    btScalar mass_;
    btVector3 inertia_;

    // the list of shapes obtained from the scene graph.
    std::vector<ShapeElement> shapes_;

    btCompoundShape* bullet_compound_shape_ = nullptr;
    btEmptyShape* bullet_empty_shape_ = nullptr;

    // stores last transform acquired by get_transform(). Useful for checking
    // whether bounding boxes needs to be updated.
    mutable btTransform last_body_transform_;
};

} // namespace physics
} // namespace gua

#endif // GUA_RIGID_BODY_NODE_HPP
