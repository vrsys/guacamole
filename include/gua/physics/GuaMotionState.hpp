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

#ifndef GUA_MOTION_STATE_HPP
#define GUA_MOTION_STATE_HPP

// external headers
#include <LinearMath/btMotionState.h>
#include <utility>

namespace gua
{
namespace physics
{
class RigidBodyNode;

/**
 * The GuaMotionState is an implementation of Bullet's btMotionState.
 *        This class is used internally in RigidBodyNode and Physics classes
 *        for synchronizing world transforms.
 */
class GuaMotionState : public btMotionState
{
  public:
    /**
     * Constructor.
     *
     * Creates a new motion state instance.
     *
     * \param start_trans Initial transform.
     */
    GuaMotionState(const btTransform& start_trans = btTransform::getIdentity());

    /**
     * Destructor.
     *
     * Deletes the motion state and frees all associated data.
     */
    ~GuaMotionState();

    /**
     * Synchronizes world transform from user to physics. This method
     *        is called by Bullet's simulation function.
     *
     * \param [out] centerOfMassWorldTrans Output where the current
     *                                     transform should be stored.
     */
    virtual void getWorldTransform(btTransform& centerOfMassWorldTrans) const;

    /**
     * Synchronizes world transform from physics to user. This method
     *        is called by Bullet's simulation function for active
     *        rigid bodies.
     *
     * \param centerOfMassWorldTrans New transform.
     */
    virtual void setWorldTransform(const btTransform& centerOfMassWorldTrans);

    /**
     * Gets the latest world transform that becomes available after
     *        calling flip_reader().
     *        This method is called by RigidBodyNode::get_transform().
     *
     * \param [out] centerOfMassWorldTrans Output where transform should be
     *                                     stored.
     * \sa    flip_reader()
     */
    virtual void latest_transform(btTransform& centerOfMassWorldTrans) const;

    /**
     * Copies the transform from the writer's buffer to the intermediate
     *        buffer. This method is used in Physics::simulate().
     */
    inline void flip_writer()
    {
        *transforms_[1] = *transforms_[0];
        dirty = true;
    }

    /**
     * Copies the transform from the intermediate buffer to the reader's
     *        buffer. Physics::synchronize() calls this method for all
     *        rigid bodies to retrieve the latest available transforms.
     * \sa    latest_transform()
     */
    inline void flip_reader()
    {
        if(dirty)
        {
            std::swap(transforms_[1], transforms_[2]);
            dirty = false;
        }
    }

  private:
    btTransform* transforms_[3];
    bool dirty;
};

} // namespace physics
} // namespace gua

#endif // GUA_MOTION_STATE_HPP
