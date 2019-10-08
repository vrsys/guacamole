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

#ifndef GUA_PROFILER_HPP
#define GUA_PROFILER_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>

// external headers
#include <scm/core/time/accum_timer.h>
#include <scm/gl_util/utilities.h>
#include <string>

namespace gua
{
/**
 *
 *
 *
 */
class GUA_DLL Profiler
{
  public:
    class Timer
    {
      public:
        Timer(Profiler const& profiler, std::string const& name);

        Timer(Profiler const& profiler, std::string const& name, RenderContext const& context);

      private:
        scm::gl::util::scoped_timer timer_;
    };

    Profiler();

    void enable(bool enable);

    void set_interval(int interval);

    void update();

    void print();
    void print(std::vector<std::string> const& names);

    friend class Timer;

  private:
    scm::gl::util::profiling_host_ptr profile_host_;
    mutable std::set<std::string> timer_names_;

    int interval_;
    int frame_count_;
};

} // namespace gua
#endif // GUA_PROFILER_HPP
