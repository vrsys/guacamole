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
#include <gua/utils/Profiler.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

Profiler::Timer::Timer(Profiler const& profiler, std::string const& name) : timer_(*profiler.profile_host_, name)
{
    if(profiler.profile_host_->enabled())
        profiler.timer_names_.insert(name);
}

////////////////////////////////////////////////////////////////////////////////

Profiler::Timer::Timer(Profiler const& profiler, std::string const& name, RenderContext const& context) : timer_(*profiler.profile_host_, name, context.render_context)
{
    if(profiler.profile_host_->enabled())
        profiler.timer_names_.insert(name);
}

////////////////////////////////////////////////////////////////////////////////

Profiler::Profiler() : profile_host_(scm::make_shared<scm::gl::util::profiling_host>()), timer_names_(), interval_(500), frame_count_(0) {}

////////////////////////////////////////////////////////////////////////////////

void Profiler::enable(bool enable) { profile_host_->enabled(enable); }

////////////////////////////////////////////////////////////////////////////////

void Profiler::update() { profile_host_->update(interval_); }

////////////////////////////////////////////////////////////////////////////////

void Profiler::set_interval(int interval) { interval_ = interval; }

////////////////////////////////////////////////////////////////////////////////

void Profiler::print()
{
    if(profile_host_->enabled() && ++frame_count_ > interval_)
    {
        Logger::LOG_MESSAGE << "#### start of profiling information ####" << std::endl;

        for(auto& name : timer_names_)
        {
            std::cout << name << ": " << scm::gl::util::profiling_result(profile_host_, name, scm::time::time_io(scm::time::time_io::msec, 3)) << std::endl;
        }

        frame_count_ = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Profiler::print(std::vector<std::string> const& names)
{
    if(profile_host_->enabled() && ++frame_count_ > interval_)
    {
        Logger::LOG_MESSAGE << "#### start of profiling information ####" << std::endl;

        for(auto& name : names)
        {
            std::cout << name << ": " << scm::gl::util::profiling_result(profile_host_, name, scm::time::time_io(scm::time::time_io::msec, 3)) << std::endl;
        }

        frame_count_ = 0;
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
