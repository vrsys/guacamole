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

#ifndef SIGNAL_HPP_
#define SIGNAL_HPP_

#include <map>
#include <algorithm>
#include <functional>
#include <gua/platform.hpp>

namespace gua
{
namespace events
{
template <typename... Parameters>
class Signal
{
  public:
    Signal() : current_id_(0) {}

    template <typename F, typename... Args>
    int connect_member(F&& f, Args&&... a) const
    {
        callbacks_.insert(std::make_pair(++current_id_, std::bind(f, a...)));
        return current_id_;
    }

    int connect(std::function<void(Parameters...)> const& callback) const
    {
        callbacks_.insert(std::make_pair(++current_id_, callback));
        return current_id_;
    }

    void disconnect(int id) const { callbacks_.erase(id); }

    void emit(Parameters... p)
    {
        for(auto& callback : callbacks_)
        {
            callback.second(p...);
        }
    }

  private:
    mutable std::map<int, std::function<void(Parameters...)>> callbacks_;
    mutable int current_id_;
};

} // namespace events
} // namespace gua

#endif /* SIGNAL_HPP_ */
