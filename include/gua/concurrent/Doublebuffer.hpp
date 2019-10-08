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

#ifndef GUA_DOUBLEBUFFER_HPP
#define GUA_DOUBLEBUFFER_HPP

#include <atomic>
#include <mutex>
#include <thread>
#include <iostream>
#include <condition_variable>
#include <boost/optional.hpp>
#include <boost/none.hpp>

namespace gua
{
namespace concurrent
{
template <typename T>
class Doublebuffer
{
  public:
    Doublebuffer() : front_(), back_(), updated_(false), copy_mutex_(), copy_cond_var_(), shutdown_(false) {}

    bool push_back(T const& scene_graphs)
    {
        if(shutdown_)
            return false;
        {
            // blocks until ownership can be obtained for the current thread.
            std::lock_guard<std::mutex> lock(copy_mutex_);
            back_ = scene_graphs;
            updated_ = true;
        }
        copy_cond_var_.notify_one();
        return true;
    }

    boost::optional<T> read()
    {
        if(shutdown_)
        {
            return boost::optional<T>();
        }
        {
            std::unique_lock<std::mutex> lock(copy_mutex_);
            while(!updated_ && !shutdown_)
            {
                copy_cond_var_.wait(lock);
            }
            if(shutdown_)
            {
                return boost::none;
            }
            updated_ = false;
            std::swap(front_, back_);
        }

        return boost::make_optional(front_);
    }
    inline void close()
    {
        shutdown_ = true;
        copy_cond_var_.notify_all();
    }

    bool closed() const { return shutdown_; }

  private:
    T front_;
    T back_;
    bool updated_;

    std::mutex copy_mutex_;
    std::condition_variable copy_cond_var_;
    std::atomic<bool> shutdown_;
};

} // namespace concurrent

} // namespace gua

#endif // GUA_DOUBLEBUFFER_HPP
