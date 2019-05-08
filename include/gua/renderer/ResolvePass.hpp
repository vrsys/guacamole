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

#ifndef GUA_RESOLVE_PASS_HPP
#define GUA_RESOLVE_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua
{
class Pipeline;

class GUA_DLL ResolvePassDescription : public PipelinePassDescription
{
  public:
    enum class BackgroundMode
    {
        COLOR = 0,
        SKYMAP_TEXTURE = 1,
        QUAD_TEXTURE = 2,
        CUBEMAP = 3
    };

    enum class ToneMappingMethod
    {
        LINEAR = 0,
        HEJL = 1,
        REINHARD = 2,
        UNCHARTED = 3
    };

    enum class EnvironmentLightingMode
    {
        SPHEREMAP = 0,
        CUBEMAP = 1,
        AMBIENT_COLOR = 2
    };

    ResolvePassDescription();

    /////////////////////////////////////////////////////////////////////////////
    // background
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& background_mode(BackgroundMode mode);
    BackgroundMode background_mode() const;

    ResolvePassDescription& background_color(utils::Color3f const& color);
    utils::Color3f background_color() const;

    ResolvePassDescription& background_texture(std::string const& texture);
    std::string background_texture() const;

    ResolvePassDescription& alternative_background_texture(std::string const& texture);
    std::string alternative_background_texture() const;

    ResolvePassDescription& background_texture_blend_factor(float factor);
    float background_texture_blend_factor() const;

    /////////////////////////////////////////////////////////////////////////////
    // ambient lighting
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& environment_lighting_texture(std::string const& texture);
    std::string const& environment_lighting_texture() const;

    ResolvePassDescription& alternative_environment_lighting_texture(std::string const& texture);
    std::string const& alternative_environment_lighting_texture() const;

    ResolvePassDescription& environment_lighting_texture_blend_factor(float factor);
    float environment_lighting_texture_blend_factor() const;

    ResolvePassDescription& environment_lighting(utils::Color3f const& color);
    utils::Color3f environment_lighting() const;

    ResolvePassDescription& environment_lighting_mode(EnvironmentLightingMode mode);
    EnvironmentLightingMode environment_lighting_mode() const;

    ResolvePassDescription& horizon_fade(float value);
    float horizon_fade() const;

    /////////////////////////////////////////////////////////////////////////////
    // screen-space ambient occlusion
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& ssao_enable(bool enable);
    bool ssao_enable() const;

    ResolvePassDescription& ssao_radius(float radius);
    float ssao_radius() const;

    ResolvePassDescription& ssao_intensity(float intensity);
    float ssao_intensity() const;

    ResolvePassDescription& ssao_falloff(float falloff);
    float ssao_falloff() const;

    ResolvePassDescription& ssao_noise_texture(std::string const& texture);
    std::string const& ssao_noise_texture() const;

    /////////////////////////////////////////////////////////////////////////////
    // screen-space shadows
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& screen_space_shadows(bool enable);
    bool screen_space_shadows() const;

    ResolvePassDescription& screen_space_shadow_radius(float radius_in_world_space_units);
    float screen_space_shadow_radius() const;

    ResolvePassDescription& screen_space_shadow_max_radius_px(float max_radius_in_pixel);
    float screen_space_shadow_max_radius_px() const;

    ResolvePassDescription& screen_space_shadow_intensity(float intensity);
    float screen_space_shadow_intensity() const;

    /////////////////////////////////////////////////////////////////////////////
    // fog
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& enable_fog(bool enable_fog);
    bool enable_fog() const;

    ResolvePassDescription& fog_start(float fog_start);
    float fog_start() const;

    ResolvePassDescription& fog_end(float fog_end);
    float fog_end() const;

    /////////////////////////////////////////////////////////////////////////////
    // vignette
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& vignette_coverage(float vignette_coverage);
    float vignette_coverage() const;

    ResolvePassDescription& vignette_softness(float vignette_softness);
    float vignette_softness() const;

    ResolvePassDescription& vignette_color(math::vec4f const& vignette_color);
    math::vec4f vignette_color() const;

    /////////////////////////////////////////////////////////////////////////////
    // tone mapping and exposure
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& tone_mapping_exposure(float value);
    float tone_mapping_exposure() const;

    ResolvePassDescription& tone_mapping_method(ToneMappingMethod value);
    ToneMappingMethod tone_mapping_method() const;

    /////////////////////////////////////////////////////////////////////////////
    // debug view
    /////////////////////////////////////////////////////////////////////////////
    ResolvePassDescription& debug_tiles(bool value);
    bool debug_tiles() const;

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

  protected:
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;

    ToneMappingMethod tone_mapping_method_ = ToneMappingMethod::LINEAR;
    bool debug_tiles_ = false;
};
} // namespace gua

#endif // GUA_RESOLVE_PASS_HPP
