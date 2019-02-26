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

/**
 * \file
 * Declaration of the SpinLock class.
 */
#ifndef GUA_SPIN_LOCK_HPP
#define GUA_SPIN_LOCK_HPP

#include <gua/platform.hpp>

// external headers
#include <atomic>
#include <thread>

// If 1, enables rescheduling of threads when a spinlock is acquired
#define SPINLOCK_ENABLE_THREAD_YIELDING 1

namespace gua
{
/**
 * This class represents a simple spinlock.
 *
 * The SpinLock class implements BasicLockable concept, so it can be safely
 * used for scoped blocks, for example by means of std::lock_guard.
 *
 * Since spinlocks use busy-waiting technique for acquiring the lock, use
 * them only for a short period blocking.
 */

class SpinLock
{
  public:
    /**
     * Constructor.
     *
     * Creates a new spin lock.
     */
    SpinLock() : lk() { lk.clear(); }

    /**
     * Acquires the lock.
     *
     */
    inline void lock()
    {
        while(lk.test_and_set(std::memory_order_acquire))
        {
#if SPINLOCK_ENABLE_THREAD_YIELDING
            std::this_thread::yield();
#endif
        }
    }

    /**
     * Tries to acquire the lock. Returns immediately.
     * \return true if the lock was acquired successfully, otherwise false.
     */
    inline bool try_lock() { return !lk.test_and_set(std::memory_order_acquire); }

    /**
     * Releases the lock.
     *
     */
    inline void unlock() { lk.clear(std::memory_order_release); }

    // No copying construction. No assignment.

    SpinLock(const SpinLock&) = delete;
    SpinLock& operator=(const SpinLock&) = delete;

  private:
    std::atomic_flag lk;
};

} // namespace gua

#endif // GUA_SPIN_LOCK_HPP
