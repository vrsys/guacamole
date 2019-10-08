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

#include <gua/events/Ticker.hpp>

#include <boost/bind.hpp>

namespace gua
{
namespace events
{
Ticker::Ticker(MainLoop& mainloop, double tick_time) : timer_(mainloop.io_service, boost::posix_time::microseconds(int(1000000.0 * tick_time))), tick_time_(tick_time) { async_wait(); }

void Ticker::set_tick_time(double tick_time) { tick_time_ = tick_time; }

double Ticker::get_tick_time() const { return tick_time_; }

void Ticker::self_callback(int revents)
{
    async_wait();
    on_tick.emit();
}

void Ticker::async_wait()
{
    timer_.expires_from_now(boost::posix_time::microseconds(int(1000000.0 * tick_time_)));
    timer_.async_wait(boost::bind(&Ticker::self_callback, this, 0));
}

} // namespace events
} // namespace gua
