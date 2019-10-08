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
#include <gua/physics/Physics.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/physics/CollisionShapeNode.hpp>
#include <gua/databases/CollisionShapeDatabase.hpp>
#include <gua/physics/GuaMotionState.hpp>
#include <gua/physics/Constraint.hpp>
#include <gua/physics/PhysicsUtils.hpp>
#include <gua/renderer/DisplayData.hpp>

// external headers
#include <iostream>
#include <stack>

using std::lock_guard;
using std::mutex;
using std::thread;
namespace this_thread = std::this_thread;
namespace chrono = std::chrono;

namespace gua
{
namespace physics
{
Physics::Physics()
    : simulation_mutex_(), start_stop_mutex_(), motion_state_update_mutex_(), pause_mutex_(), thread_(nullptr), is_stopped_(true), fixed_timestep_(physics_default_fixed_timestep),
      reduce_sim_rate_(physics_default_reduce_sim_rate), max_sim_time_(chrono::microseconds(physics_default_max_sim_time)), shape_visitor_(), call_once_queue_(), call_once_queue_mutex_(),
      broadphase_(new btDbvtBroadphase()), collision_configuration_(new btDefaultCollisionConfiguration()), dispatcher_(new btCollisionDispatcher(collision_configuration_)),
      solver_(new btSequentialImpulseConstraintSolver()), dw_(new btDiscreteDynamicsWorld(dispatcher_, broadphase_, solver_, collision_configuration_)), rigid_bodies_(), constraints_(),
      physics_fps_(0.0f)
{
    if(dw_)
        dw_->setGravity(btVector3(0, physics_default_gravity, 0));
}

////////////////////////////////////////////////////////////////////////////////

Physics::Physics(float gravity, float fixed_timestep)
    : simulation_mutex_(), start_stop_mutex_(), motion_state_update_mutex_(), pause_mutex_(), thread_(nullptr), is_stopped_(true), fixed_timestep_(fixed_timestep),
      reduce_sim_rate_(physics_default_reduce_sim_rate), max_sim_time_(chrono::microseconds(physics_default_max_sim_time)), shape_visitor_(), call_once_queue_(), call_once_queue_mutex_(),
      broadphase_(new btDbvtBroadphase()), collision_configuration_(new btDefaultCollisionConfiguration()), dispatcher_(new btCollisionDispatcher(collision_configuration_)),
      solver_(new btSequentialImpulseConstraintSolver()), dw_(new btDiscreteDynamicsWorld(dispatcher_, broadphase_, solver_, collision_configuration_)), rigid_bodies_(), constraints_(),
      physics_fps_(0.0f)
{
    if(dw_)
        dw_->setGravity(btVector3(0, gravity, 0));

    // Collision calback example
    /*
    dw_->setInternalTickCallback(
        [](btDynamicsWorld *world, btScalar timeStep) {
        int numManifolds = world->getDispatcher()->getNumManifolds();
        for (int i=0; i<numManifolds; i++) {
            btPersistentManifold* contactManifold = world->getDispatcher()->
                                                  getManifoldByIndexInternal(i);
            const btCollisionObject* obA =
             static_cast<const btCollisionObject*>(contactManifold->getBody0());
            const btCollisionObject* obB =
             static_cast<const btCollisionObject*>(contactManifold->getBody1());

            int numContacts = contactManifold->getNumContacts();
            for (int j=0; j<numContacts; j++) {
                btManifoldPoint& pt = contactManifold->getContactPoint(j);

                if (pt.getDistance()<0.f) {
                    const btVector3& ptA = pt.getPositionWorldOnA();
                    const btVector3& ptB = pt.getPositionWorldOnB();
                    const btVector3& normalOnB = pt.m_normalWorldOnB;

                    std::cout
                        << "Col. A:" << obA << " ["
                        << ptA.x() << ',' << ptA.y() << ',' << ptA.z() << "]"
                        << std::endl
                        << "     B:" << obB << " ["
                        << ptB.x() << ',' << ptB.y() << ',' << ptB.z() << "]"
                        << std::endl;

                }
            }
        }
    }); //*/
}

////////////////////////////////////////////////////////////////////////////////

Physics::~Physics()
{
    stop_simulation();
    while(constraints_.size())
        remove_constraint(constraints_[0]);
    while(rigid_bodies_.size())
        remove_rigid_body(rigid_bodies_[0]);
    delete dw_;
    delete solver_;
    delete collision_configuration_;
    delete dispatcher_;
    delete broadphase_;
}

////////////////////////////////////////////////////////////////////////////////

void Physics::start_simulation()
{
    lock_guard<mutex> lk(start_stop_mutex_);
    if(is_stopped_.load())
    {
        is_stopped_.store(false);
        thread_ = new thread([&] { simulate(); });
    }
}

////////////////////////////////////////////////////////////////////////////////

void Physics::stop_simulation()
{
    lock_guard<mutex> lk(start_stop_mutex_);
    if(!is_stopped_.load())
    {
        is_stopped_.store(true);
        thread_->join();
        delete thread_;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Physics::pause(bool pause)
{
    if(is_stopped_.load())
        return;

    if(pause)
        pause_mutex_.try_lock();
    else
        pause_mutex_.unlock();
}

////////////////////////////////////////////////////////////////////////////////

void Physics::set_gravity(math::vec3 const& gravity) { dw_->setGravity(math::vec3_to_btVector3(gravity)); }

////////////////////////////////////////////////////////////////////////////////

math::vec3 Physics::get_gravity() const { return math::btVector3_to_vec3(dw_->getGravity()); }

////////////////////////////////////////////////////////////////////////////////

void Physics::set_fixed_timestep(float step)
{
    lock_guard<mutex> l(simulation_mutex_);
    fixed_timestep_ = step;
}

////////////////////////////////////////////////////////////////////////////////

void Physics::simulation_rate_reduction(bool enabled, int max_sim_time)
{
    lock_guard<mutex> lk(simulation_mutex_);
    reduce_sim_rate_ = enabled, max_sim_time_ = chrono::microseconds(max_sim_time);
}

////////////////////////////////////////////////////////////////////////////////

void Physics::synchronize(bool auto_start)
{
    // swap motion state transforms^
    {
        lock_guard<SpinLock> lk(motion_state_update_mutex_);

        for(auto rb : rigid_bodies_)
        {
            rb->motion_state_->flip_reader();
        }
    }

    // traverse rigid body subgraphs in order to apply collision shapes
    for(auto& rb : rigid_bodies_)
    {
        shape_visitor_.check(&*rb);
    }

    if(auto_start && is_stopped_.load())
        start_simulation();
}

////////////////////////////////////////////////////////////////////////////////

void Physics::add_rigid_body(std::shared_ptr<RigidBodyNode> const& body)
{
    if(body && std::find(rigid_bodies_.begin(), rigid_bodies_.end(), body) == rigid_bodies_.end())
    {
        lock_guard<mutex> lk(simulation_mutex_);
        dw_->addRigidBody(body->body_);
        rigid_bodies_.push_back(body);
        body->ph_ = this;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Physics::add_rigid_body(std::shared_ptr<RigidBodyNode> const& body, RigidBodyNode::CollisionFilterGroups const& group, RigidBodyNode::CollisionFilterGroups const& mask)
{
    if(body && std::find(rigid_bodies_.begin(), rigid_bodies_.end(), body) == rigid_bodies_.end())
    {
        lock_guard<mutex> lk(simulation_mutex_);
        dw_->addRigidBody(body->body_, static_cast<btBroadphaseProxy::CollisionFilterGroups>(group), static_cast<btBroadphaseProxy::CollisionFilterGroups>(mask));
        rigid_bodies_.push_back(body);
        body->ph_ = this;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Physics::remove_rigid_body(std::shared_ptr<RigidBodyNode> const& body)
{
    auto i = std::find(rigid_bodies_.begin(), rigid_bodies_.end(), body);
    if(i != rigid_bodies_.end())
    {
        lock_guard<mutex> lk(simulation_mutex_);
        dw_->removeRigidBody(body->body_);
        (*i)->ph_ = nullptr;
        rigid_bodies_.erase(i);
    }
}

////////////////////////////////////////////////////////////////////////////////

void Physics::add_constraint(Constraint* ct, bool disable_collisions_between_linked_bodies)
{
    if(ct && ct->ct_ && std::find(constraints_.begin(), constraints_.end(), ct) == constraints_.end())
    {
        lock_guard<mutex> lk(simulation_mutex_);
        dw_->addConstraint(ct->ct_, disable_collisions_between_linked_bodies);
        constraints_.push_back(ct);
        ct->ph_ = this;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Physics::remove_constraint(Constraint* ct)
{
    auto i = std::find(constraints_.begin(), constraints_.end(), ct);
    if(i != constraints_.end())
    {
        lock_guard<mutex> lk(simulation_mutex_);
        dw_->removeConstraint(ct->ct_);
        (*i)->ph_ = nullptr;
        constraints_.erase(i);
    }
}

////////////////////////////////////////////////////////////////////////////////

float Physics::get_physics_fps() const
{
    // return physics_fps_.load();
    return physics_fps_;
}

////////////////////////////////////////////////////////////////////////////////

void Physics::call_once(std::function<void()> fun)
{
    lock_guard<mutex> lk(call_once_queue_mutex_);
    call_once_queue_.push(fun);
}

////////////////////////////////////////////////////////////////////////////////

void Physics::simulate()
{
    auto current_time = chrono::high_resolution_clock::now();
    auto last_time = chrono::high_resolution_clock::now();

    while(!is_stopped_.load())
    {
        current_time = chrono::high_resolution_clock::now();

        // Process call-once queue
        std::function<void()> fun;
        while(pop_call_once(fun))
            fun();

        {
            lock_guard<mutex> l(simulation_mutex_);
            // Validate constraints before simulation
            std::for_each(constraints_.begin(), constraints_.end(), std::mem_fn(&Constraint::validate));
            // for (auto ct: constraints_) ct->validate();

            const btScalar current_timestep = chrono::duration_cast<chrono::microseconds>(current_time - last_time).count() * 1.e-6f;

            dw_->stepSimulation(current_timestep, physics_default_max_sub_steps, fixed_timestep_);
            // Swap motion state transforms
            {
                lock_guard<SpinLock> lk(motion_state_update_mutex_);
                for(auto rb : rigid_bodies_)
                {
                    rb->motion_state_->flip_writer();
                }
            }
        }
        if(reduce_sim_rate_)
        {
            auto sim_t = chrono::high_resolution_clock::now() - current_time;
            if(sim_t < max_sim_time_)
                this_thread::sleep_for(max_sim_time_ - sim_t);
        }

        // Pause
        {
            lock_guard<SpinLock> lk(pause_mutex_);
        }

        // physics_fps_.store(1.e6f/(std::chrono::duration_cast<std::chrono::microseconds>(
        //      std::chrono::high_resolution_clock::now()-current_time).count()));
        physics_fps_ = 1.e6f / (chrono::duration_cast<chrono::microseconds>(chrono::high_resolution_clock::now() - current_time).count());

        DisplayData dd;
        dd.set_physics_fps(physics_fps_);

        last_time = current_time;
    }
}

////////////////////////////////////////////////////////////////////////////////

bool Physics::pop_call_once(std::function<void()>& value)
{
    lock_guard<mutex> lk(call_once_queue_mutex_);
    if(call_once_queue_.empty())
        return false;
    value = call_once_queue_.front();
    call_once_queue_.pop();
    return true;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace physics
} // namespace gua
