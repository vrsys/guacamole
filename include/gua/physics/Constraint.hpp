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

#ifndef CONSTRAINT_HPP
#define CONSTRAINT_HPP

// guacamole headers
#include <gua/utils/SpinLock.hpp>
#include <gua/math/math.hpp>

// external headers
#include <utility>
#include <gua/platform.hpp>

#if GUA_COMPILER == GUA_COMPILER_MSVC
#include <boost/smart_ptr/detail/spinlock.hpp>
#include <boost/atomic.hpp>
#else
#include <atomic>
#endif

#include <BulletDynamics/ConstraintSolver/btTypedConstraint.h>

namespace gua {
namespace physics {

class Physics;
class RigidBodyNode;

/**
 * This abstract class is the base class for rigid body constraints.
 *
 */
class GUA_DLL Constraint {
  friend class Physics;

 public:

  /**
   * Constructor.
   *
   * Creates a new constraint between two rigid bodies.
   *
   * \param body_a The first rigid body.
   * \param body_b The second rigid body (if nullptr, the constraint will be
   *               created between the first rigid body and a fixed point).
   */
  Constraint(RigidBodyNode* body_a, RigidBodyNode* body_b = nullptr);

  /**
   * Destructor.
   *
   * Deletes the constraint and frees all associated data.
   */
  virtual ~Constraint();

  /**
   * Returns the first rigid body linked with the constraint.
   *
   * \return The pointer to the scene graph's rigid body node.
   */
  RigidBodyNode* body_a() const { return body_a_; }

  void set_body_a(RigidBodyNode* body_a) { body_a_ = body_a; }

  /**
   * Returns the second rigid body linked with the constraint.
   *
   * \return The pointer to the scene graph's rigid body node (nullptr if the
   *         constraint linked with only one rigid body).
   */
  RigidBodyNode* body_b() const { return body_b_; }

  void set_body_b(RigidBodyNode* body_b) { body_b_ = body_b; }

  /**
   * Sets the status of the constraint.
   *
   * \param enabled Set to false to deactivate the constraint between
   *                linked bodies.
   */
  void set_enabled(bool enabled);

  /**
   * Returns the status of the constraint.
   *
   * \return True if the constaraint is enabled.
   */
  bool enabled() const { return ct_->isEnabled(); }

  /**
   * Sets the current breaking impulse threshold.
   *
   * \param threshold Minimum impulse applied to the constraint when the
   *                  constraint gets disabled.
   */
  void set_breaking_impulse_threshold(float threshold);

  /**
   * Returns the current breaking impulse threshold.
   *
   */
  float breaking_impulse_threshold() const {
    return breaking_impulse_threshold_.first;
  }

// No copying construction. No assignment.
#if GUA_COMPILER == GUA_COMPILER_MSVC
 private:
  Constraint(const Constraint&);
  Constraint& operator=(const Constraint&);
#else
  Constraint(const Constraint&) = delete;
  Constraint& operator=(const Constraint&) = delete;
#endif

 protected:

  /**
   * Applies changes to the Bullet constraint.
   *
   * This method is called by Physics class instance before each
   * simulation step.
   */
  void validate();

  /**
   * Pure virtual method that is called by validate()
   *
   * This method should be implemented in derived constraint classes.
   */
  virtual void update_constraint() = 0;

  typedef std::pair<math::mat4, bool> Mat4Field;
  typedef std::pair<math::vec3, bool> Vec3Field;
  typedef std::pair<float, bool> FloatField;
  typedef std::pair<bool, bool> BoolField;

  SpinLock lock_;

#if GUA_COMPILER == GUA_COMPILER_MSVC
  boost::atomic<bool> invalid_;
#else
  std::atomic_bool invalid_;
#endif

  Physics* ph_;
  btTypedConstraint* ct_;
  RigidBodyNode* body_a_;
  RigidBodyNode* body_b_;
  BoolField enabled_;
  FloatField breaking_impulse_threshold_;
};

}
}

#endif  // CONSTRAINT_HPP
