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

#ifndef GUA_SKY_MAP_PASS
#define GUA_SKY_MAP_PASS

#include <gua/renderer/PipelinePass.hpp>

#include <gua/platform.hpp>

// external headers
#include <scm/gl_core/buffer_objects.h>

namespace gua
{
class GUA_DLL SkyMapPassDescription : public PipelinePassDescription
{
  public:
    SkyMapPassDescription();
    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

    SkyMapPassDescription& light_direction(math::vec3f const& light_direction);
    math::vec3f light_direction() const;

    SkyMapPassDescription& light_color(math::vec3f const& light_color);
    math::vec3f light_color() const;

    SkyMapPassDescription& light_brightness(float light_brightness);
    float light_brightness() const;

    SkyMapPassDescription& ground_color(math::vec3f const& ground_color);
    math::vec3f ground_color() const;

    SkyMapPassDescription& rayleigh_factor(float rayleigh_factor);
    float rayleigh_factor() const;

    SkyMapPassDescription& mie_factor(float mie_factor);
    float mie_factor() const;

    SkyMapPassDescription& output_texture_name(std::string const& output_texture_name);
    std::string output_texture_name() const;

  protected:
    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;
};

} // namespace gua

#endif // GUA_SKY_MAP_PASS
