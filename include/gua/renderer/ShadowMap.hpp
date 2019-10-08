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

#ifndef GUA_SHADOW_MAP_HPP
#define GUA_SHADOW_MAP_HPP

// guacamole headers
#include <gua/renderer/RenderTarget.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/utils/Mask.hpp>

namespace gua
{
namespace node
{
class LightNode;
}

class Pipeline;
struct CachedShadowMap;

/**
 *
 */
class ShadowMap : public RenderTarget
{
  public:
    ShadowMap(RenderContext const& ctx, math::vec2ui const& resolution);

    virtual void clear(RenderContext const& context, float depth = 1.f, unsigned stencil = 0) override;
    virtual void bind(RenderContext const& context, bool write_depth) override;

    virtual void set_viewport(RenderContext const& context) override;
    void set_viewport_offset(math::vec2f const& offset);
    void set_viewport_size(math::vec2f const& size);

    virtual void remove_buffers(RenderContext const& ctx) override;

    scm::gl::texture_2d_ptr const& get_depth_buffer() const override;

  private:
    scm::gl::frame_buffer_ptr fbo_;
    scm::gl::texture_2d_ptr depth_buffer_;
    scm::gl::sampler_state_ptr sampler_state_;
    math::vec2f viewport_offset_;
    math::vec2f viewport_size_;
};

struct CachedShadowMap
{
    std::shared_ptr<ShadowMap> shadow_map;
    Mask render_mask;
};

struct SharedShadowMapResource
{
    std::set<std::shared_ptr<ShadowMap>> unused_shadow_maps;
    std::unordered_map<node::LightNode*, std::vector<CachedShadowMap>> used_shadow_maps;
};

} // namespace gua

#endif // GUA_SHADOW_MAP_HPP
