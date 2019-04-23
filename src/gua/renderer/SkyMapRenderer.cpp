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
#include <gua/renderer/SkyMapRenderer.hpp>

#include <gua/renderer/ResourceFactory.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/TextureDatabase.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

SkyMapRenderer::SkyMapRenderer() : program_(nullptr), sky_map_(nullptr), fbo_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

void SkyMapRenderer::render_sky_map(Pipeline& pipe, PipelinePassDescription const& desc)
{
    auto const& ctx(pipe.get_context());
    const int size_(1024);

    auto tex_uniform(desc.uniforms.find("output_texture_name"));
    auto output_texture_name(boost::get<std::string>(tex_uniform->second.data));

    auto light_dir_uniform(desc.uniforms.find("light_direction"));
    auto light_direction(boost::get<math::vec3f>(light_dir_uniform->second.data));

    auto light_color_uniform(desc.uniforms.find("light_color"));
    auto light_color(boost::get<math::vec3f>(light_color_uniform->second.data));

    auto light_brightness_uniform(desc.uniforms.find("light_brightness"));
    auto light_brightness(boost::get<float>(light_brightness_uniform->second.data));

    auto ground_color_uniform(desc.uniforms.find("ground_color"));
    auto ground_color(boost::get<math::vec3f>(ground_color_uniform->second.data));

    auto rayleigh_factor_uniform(desc.uniforms.find("rayleigh_factor"));
    auto rayleigh_factor(boost::get<float>(rayleigh_factor_uniform->second.data));

    auto mie_factor_uniform(desc.uniforms.find("mie_factor"));
    auto mie_factor(boost::get<float>(mie_factor_uniform->second.data));

    math::vec3f rayleigh_mie_light_brightness(rayleigh_factor, mie_factor, light_brightness);

    if(!sky_map_)
    {
        if(output_texture_name == "")
        {
            gua::Logger::LOG_WARNING << "Unable to render skymap: No output texture name specified!" << std::endl;
            return;
        }

        if(TextureDatabase::instance()->contains(output_texture_name))
        {
            sky_map_ = std::dynamic_pointer_cast<TextureCube>(TextureDatabase::instance()->lookup(output_texture_name));
        }
        else
        {
            sky_map_ = std::make_shared<TextureCube>(size_, size_, scm::gl::FORMAT_RGB_32F, 1);
            TextureDatabase::instance()->add(output_texture_name, sky_map_);
        }
    }

    if(!fbo_)
    {
        fbo_ = ctx.render_device->create_frame_buffer();
        fbo_->attach_color_buffer(0, sky_map_->get_buffer(ctx), 0, 0);
    }

    if(!program_)
    {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
        ResourceFactory factory;
        std::string v_shader = factory.read_shader_file("resources/shaders/atmospheric_scattering.vert");
        std::string g_shader = factory.read_shader_file("resources/shaders/atmospheric_scattering.geom");
        std::string f_shader = factory.read_shader_file("resources/shaders/atmospheric_scattering.frag");
#else
        std::string v_shader = Resources::lookup_shader("shaders/atmospheric_scattering.vert");
        std::string g_shader = Resources::lookup_shader("shaders/atmospheric_scattering.geom");
        std::string f_shader = Resources::lookup_shader("shaders/atmospheric_scattering.frag");
#endif

        program_ = std::make_shared<ShaderProgram>();
        program_->create_from_sources(v_shader, g_shader, f_shader);
    }

    ctx.render_context->set_frame_buffer(fbo_);
    ctx.render_context->set_viewport(scm::gl::viewport(math::vec2ui(0, 0), math::vec2ui(size_, size_)));

    program_->use(ctx);
    program_->set_uniform(ctx, 1.f / size_, "texel_size");
    program_->set_uniform(ctx, light_direction, "light_direction");
    program_->set_uniform(ctx, light_color, "light_color");
    program_->set_uniform(ctx, ground_color, "ground_color");
    program_->set_uniform(ctx, rayleigh_mie_light_brightness, "rayleigh_mie_light_brightness");

    pipe.draw_quad();

    program_->unuse(ctx);
    ctx.render_context->reset_state_objects();
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
