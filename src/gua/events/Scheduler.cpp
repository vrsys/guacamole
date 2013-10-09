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

#include <gua/events/Scheduler.hpp>

namespace gua {
namespace events {

Scheduler::Scheduler() {}

Scheduler::~Scheduler() {}

void Scheduler::execute_delayed(std::function<void()> callback, double delay) {

  ev::timer* timer = new ev::timer();
  timer->set<Scheduler, &Scheduler::self_callback>(this);
  timer->start(delay, 0);

  tasks_.insert(std::make_pair(timer, callback));
}

void Scheduler::self_callback(ev::timer& timer, int revents) {
  auto callback(tasks_.find(&timer));

  if (callback != tasks_.end()) {
    callback->second();
    tasks_.erase(&timer);
    delete& timer;
  }
}

}
}
