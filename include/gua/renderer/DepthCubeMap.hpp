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

#ifndef GUA_DEPTH_CUBEMAP_HPP
#define GUA_DEPTH_CUBEMAP_HPP

// guacamole headers
#include <gua/renderer/RenderTarget.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/TextureDistance.hpp>
#include <gua/utils/Mask.hpp>

namespace gua
{
namespace node
{
class CubemapNode;
}

class Pipeline;

/**
 *
 */
class DepthCubeMap : public RenderTarget
{
  public:
    DepthCubeMap(RenderContext const& ctx, math::vec2ui const& resolution, std::string const& tex_name);

    void clear(RenderContext const& context, float depth = 1.f, unsigned stencil = 0) override;
    void bind(RenderContext const& context, bool write_depth) override;

    void set_viewport(RenderContext const& context) override;
    void set_viewport_offset(math::vec2f const& offset);
    void set_viewport_size(math::vec2f const& size);
    math::vec2f get_viewport_size() const { return viewport_size_; }

    void remove_buffers(RenderContext const& ctx) override;

    void retrieve_data(RenderContext const& ctx, float near_clip, float far_clip);

    scm::gl::texture_2d_ptr const& get_depth_buffer() const override;

  private:
    scm::gl::frame_buffer_ptr fbo_;
    std::shared_ptr<TextureDistance> texture_distance_;
    scm::gl::texture_2d_ptr depth_buffer_;
    math::vec2f viewport_offset_;
    math::vec2f viewport_size_;
};

struct SharedDepthCubeMapResource
{
    std::unordered_map<std::string, std::shared_ptr<DepthCubeMap>> cube_maps_;
};

} // namespace gua

#endif // GUA_DEPTH_CUBEMAP_HPP
