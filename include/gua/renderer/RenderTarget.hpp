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

#ifndef GUA_RENDER_TARGET_HPP
#define GUA_RENDER_TARGET_HPP

// guacamole headers
#include <gua/renderer/enums.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/Texture2D.hpp>

#include <memory>

namespace gua
{
class GUA_DLL RenderTarget
{
  public:
    RenderTarget(math::vec2ui const& resolution);
    virtual ~RenderTarget() {}

    virtual void clear(RenderContext const& context, float depth = 1.f, unsigned stencil = 0) = 0;
    virtual void set_viewport(RenderContext const& context);

    virtual void bind(RenderContext const& context, bool write_depth) = 0;
    virtual void unbind(RenderContext const& context);

    virtual void remove_buffers(RenderContext const& ctx) = 0;

    virtual scm::gl::texture_2d_ptr const& get_depth_buffer() const = 0;

    unsigned get_width() const { return resolution_.x; }
    unsigned get_height() const { return resolution_.y; }
    math::vec2ui const& get_resolution() const { return resolution_; }

  protected:
    math::vec2ui resolution_;
};

} // namespace gua

#endif // GUA_RENDER_TARGET_HPP
