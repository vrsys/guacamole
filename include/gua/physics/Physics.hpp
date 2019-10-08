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

#ifndef GUA_PHYSICS_HPP
#define GUA_PHYSICS_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/physics/CollisionShapeNodeVisitor.hpp>
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/utils/SpinLock.hpp>

#include <btBulletDynamicsCommon.h>

// external headers
#include <vector>
#include <queue>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>

// forward declarations of Bullet's classes
class btDynamicsWorld;
class btBroadphaseInterface;
class btDefaultCollisionConfiguration;
class btCollisionDispatcher;
class btSequentialImpulseConstraintSolver;

namespace gua
{
namespace physics
{
// default simulation parameters
const float physics_default_gravity = -9.81f;            // m/s
const float physics_default_fixed_timestep = 1.f / 80.f; // seconds
const int physics_default_max_sub_steps = 3;
const bool physics_default_reduce_sim_rate = true;
const int physics_default_max_sim_time = int(1.e6 / 200); // microseconds

// class RigidBodyNode;
class Constraint;

/**
 * Physics class represents a physical simulation manager.
 *
 * This class adds realistic physical simulation to guacamole. It holds all
 * rigid bodies and constraints as well as provides scene graph traversal
 * capability and methods to tune simulation parameters.
 */
class GUA_DLL Physics
{
  public:
    Physics();
    /**
     * Constructor.
     *
     * Creates a new physics manager.
     *
     * \param gravity        Global gravity in m/s. Applies to Y axis only.
     * \param fixed_timestep Fixed internal timestep in seconds. See
     *                       set_fixed_timestep() description for more details.
     */
    Physics(float gravity, float fixed_timestep);

    /**
     * Destructor.
     *
     * Deletes the physics manager and frees all associated data.
     */
    virtual ~Physics();

    /**
     * Starts physical simulation.
     *
     * Initializes a new thread that performs physical simulation.
     *
     * \sa stop_simulation(), is_running()
     */
    void start_simulation();

    /**
     * Stops physical simulation.
     *
     * Deletes the new thread that performs physical simulation.
     *
     * \sa start_simulation(), is_running(), pause()
     */
    void stop_simulation();

    /**
     * Temporarily pauses physical simulation.
     *
     * \param pause If true, simulation should be paused;
     *              if false, continue paused simulation.
     */
    void pause(bool pause);

    /**
     * Checks if the simulation is running.
     *
     * \return True, if the simulation is running. Please note that the
     *         function returns true even if the simulation thread is being
     *         initialized. Identically, it returns false when the
     *         simulation is about to stop but not yet stopped.
     */
    bool is_running() const { return !is_stopped_.load(); }

    void set_gravity(math::vec3 const& gravity);
    math::vec3 get_gravity() const;

    /**
     * Sets the fixed internal timestep of physical simulation.
     *
     * Fixed timestep indicates the frequency of performing simulation steps.
     * When the current timestep is smaller than the fixed timestep,
     * the motion is interpolated instead of be simulated.
     * According to the Bullet manual, it works best with a fixed internal
     * timestep of at least 60 hertz (1/60 second).
     *
     * \param step New fixed timestep in seconds.
     * \sa    simulation_rate_reduction()
     */
    void set_fixed_timestep(float step);

    /**
     * Gets the fixed internal timestep.
     *
     * \return Fixed internal timestep in seconds.
     */
    float get_fixed_timestep() const { return fixed_timestep_; }

    /**
     * Enables and disables reduction of simulation rate.
     *
     * This method makes it possible to save CPU resources leaving out
     * redundant simulation steps. Technically, it makes the simulation thread
     * sleep to reach max_sim_time. If time spent on simulation is greater
     * than max_sim_time, no reduction is performed.
     *
     * Note that this method does not take into account fixed internal
     * timestep. See set_fixed_timestep() description to know how fixed
     * timestep works.
     *
     * \param enabled      True if reduction should be enabled.
     * \param max_sim_time Maximum time in microseconds to be spent on each
     *                     simulation step.
     */
    void simulation_rate_reduction(bool enabled, int max_sim_time = physics_default_max_sim_time);

    /**
     * Applies new transforms to scene graph nodes and prepares
     *        collision shapes.
     *
     * This function traverses the subgraphs of all rigid bodies that are
     * under the simulation and synchronizes collision shapes associated with
     * them. This function also applies new transforms of rigid body nodes;
     * therefore, it should be called at every application step.
     *
     * \param auto_start  Automatically starts the simulation if it is not
     *                    running.
     * \sa    RigidBodyNode, CollisionShapeNode, add_rigid_body()
     */
    void synchronize(bool auto_start = false);

    /**
     * Adds new rigid body to the simulation.
     *
     * \param body Scene graph's rigid body node.
     * \sa    add_constraint()
     */
    void add_rigid_body(std::shared_ptr<RigidBodyNode> const& body);

    /**
     * Adds new rigid body to the simulation with a custom collision group and
     * a filtering mask.
     *
     * \param body Scene graph's rigid body node.
     * \param group Collision group
     * \param mask Bitwise mask for collision filtering
     * \sa    add_constraint()
     */
    void add_rigid_body(std::shared_ptr<RigidBodyNode> const& body, RigidBodyNode::CollisionFilterGroups const& group, RigidBodyNode::CollisionFilterGroups const& mask);

    /**
     * Removes a rigid body from the simulation.
     *
     * \param body Scene graph's rigid body node to be removed.
     */
    void remove_rigid_body(std::shared_ptr<RigidBodyNode> const& body);

    /**
     * Adds new constraint to the simulation.
     *
     * \param ct                                       Constraint.
     * \param disable_collisions_between_linked_bodies If true, collision
     *                         detection will be disabled for all rigid bodies
     *                         associated with the constraint.
     * \sa    add_rigid_body()
     */
    void add_constraint(Constraint* ct, bool disable_collisions_between_linked_bodies = false);

    /**
     * Removes a constraint from the simulation.
     *
     * \param ct Constraint to be removed.
     */
    void remove_constraint(Constraint* ct);

    /**
     * Gets current simulation FPS.
     *
     * \return Current FPS.
     */
    float get_physics_fps() const;

    /**
     * Invoke a given function just before the next simulation step.
     *
     * The function fun will be invoked once before the next simulation step.
     *
     * \param fun Callback function.
     */
    void call_once(std::function<void()> fun);

    /**
     * Gets the reference to the mutex that can be used to protect
     *        shared data of Physics and associated structures.
     *
     * It makes it possible to exclusively access Bullet's internal structures
     * (See get_bullet_dynamics_world(),
     * RigidBodyNode::get_bullet_rigid_body()) not allowing the physics thread
     * to perform the simulation.
     *
     * This mutex is also internally used in RigidBodyNode if it is added to
     * the simulation.
     *
     * \return The reference to the mutex.
     */
    inline std::mutex& lock() const { return simulation_mutex_; }

    ///@{
    /**
     * Gets the pointer to the internal representation of the
     *        dynamics world.
     *
     * \return The pointer to a Bullet's dynamics world.
     * \sa     lock()
     */
    const btDynamicsWorld* get_bullet_dynamics_world() const { return dw_; }
    btDynamicsWorld* get_bullet_dynamics_world() { return dw_; }
    ///@}

    // No copying construction. No assignment.
    Physics(const Physics&) = delete;
    Physics(Physics&&) = delete;
    Physics& operator=(const Physics&) = delete;
    Physics& operator=(Physics&&) = delete;

  private:
    // the simulation thread entry point.
    void simulate();

    // pop front element from call-once queue
    bool pop_call_once(std::function<void()>& value);

    // ensures exclusive access to the physics structures.
    mutable std::mutex simulation_mutex_;

    // prevents calling thread initialization/deinitialization functions at
    // the same time.
    mutable std::mutex start_stop_mutex_;

    // ensures exclusive access to buffers in motion state updates.
    mutable SpinLock motion_state_update_mutex_;

    SpinLock pause_mutex_;

    std::thread* thread_ = nullptr;
    std::atomic<bool> is_stopped_;

    float fixed_timestep_;
    bool reduce_sim_rate_;

    std::chrono::microseconds max_sim_time_;

    CollisionShapeNodeVisitor shape_visitor_;

    // queue for call-once functions
    std::queue<std::function<void()>> call_once_queue_;
    std::mutex call_once_queue_mutex_;

    // Bullet's objects
    btBroadphaseInterface* broadphase_ = nullptr;
    btDefaultCollisionConfiguration* collision_configuration_ = nullptr;
    btCollisionDispatcher* dispatcher_ = nullptr;
    btSequentialImpulseConstraintSolver* solver_ = nullptr;
    btDynamicsWorld* dw_ = nullptr;
    std::vector<std::shared_ptr<RigidBodyNode>> rigid_bodies_;
    std::vector<Constraint*> constraints_;

    /// \todo Change this to std::atomic<float> after upgrade to GCC 4.7
    float physics_fps_;
};

} // namespace physics
} // namespace gua

#endif // GUA_PHYSICS_HPP
