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
#include <gua/renderer/HeadlessSurface.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <scm/gl_core/window_management/window.h>
#include <scm/gl_core/window_management/headless_surface.h>

#include <sstream>
#include <iostream>

namespace gua
{
HeadlessSurface::HeadlessSurface(Configuration const& configuration) : WindowBase(configuration) {}

HeadlessSurface::~HeadlessSurface() { close(); }

void HeadlessSurface::open()
{
    if(window_)
    {
        window_->hide();
    }

    ctx_.context.reset();
    ctx_.display.reset();
    window_.reset();

    scm::gl::wm::surface::format_desc window_format(scm::gl::FORMAT_RGBA_8, scm::gl::FORMAT_D24_S8, true, false);

    scm::gl::wm::context::attribute_desc context_attribs(4, 4, false, config.get_debug(), false);

    ctx_.display = scm::gl::wm::display_ptr(new scm::gl::wm::display(config.get_display_name()));

    window_ = scm::gl::wm::window_ptr(new scm::gl::wm::window(ctx_.display, 0, config.get_title(), math::vec2i(0, 0), math::vec2ui(config.get_size().x, config.get_size().y), window_format));

    ctx_.context = scm::gl::wm::context_ptr(new scm::gl::wm::context(window_, context_attribs));

    headless_surface_ = scm::gl::wm::headless_surface_ptr(new scm::gl::wm::headless_surface(window_));
}

bool HeadlessSurface::get_is_open() const { return window_ != nullptr; }

bool HeadlessSurface::should_close() const { return false; }

void HeadlessSurface::close()
{
    if(get_is_open())
    {
        window_->hide();
    }

    destroy_context();
    ctx_.context.reset();
    ctx_.display.reset();
    headless_surface_.reset();
    window_.reset();
}

void HeadlessSurface::set_active(bool active)
{
    ctx_.context->make_current(window_, active);
    if(!ctx_.render_device)
    {
        init_context();
    }
}

void HeadlessSurface::finish_frame() const { window_->swap_buffers(config.get_enable_vsync()); }

} // namespace gua
