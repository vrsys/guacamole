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

#ifndef GUA_FPS_COUNTER_HPP
#define GUA_FPS_COUNTER_HPP

#include <gua/platform.hpp>
#include <gua/utils/Timer.hpp>

namespace gua
{
struct GUA_DLL FpsCounter
{
    FpsCounter(unsigned t) : fps(0.0f), frame_count(0), timer(), delay(t) {}
    void step()
    {
        if(++frame_count == delay)
        {
            fps = 1.f * delay / float(timer.get_elapsed());
            timer.reset();
            frame_count = 0;
        }
    }
    void start() { timer.start(); }

    float fps;
    unsigned frame_count;
    Timer timer;
    unsigned delay;
};

} // namespace gua

#endif // GUA_FPS_COUNTER_HPP
