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
#include <gua/renderer/PipelinePass.hpp>

#include <gua/config.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/LightTable.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
void PipelinePassDescription::touch() { ++mod_count_; }

////////////////////////////////////////////////////////////////////////////////
std::string const &PipelinePassDescription::name() const { return name_; }

////////////////////////////////////////////////////////////////////////////////
unsigned PipelinePassDescription::mod_count() const { return mod_count_; }

////////////////////////////////////////////////////////////////////////////////
PipelinePass::PipelinePass(PipelinePassDescription const &d, RenderContext const &ctx, SubstitutionMap const &substitution_map)
    : shader_(std::make_shared<ShaderProgram>()), rasterizer_state_(nullptr), depth_stencil_state_(nullptr), blend_state_(nullptr), needs_color_buffer_as_input_(d.needs_color_buffer_as_input_),
      writes_only_color_buffer_(d.writes_only_color_buffer_), enable_for_shadows_(d.enable_for_shadows_), rendermode_(d.rendermode_), process_(d.process_), name_(d.name_),
      substitution_map_(substitution_map)
{
    upload_program(d, ctx);

    if(d.depth_stencil_state_)
    {
        depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(*d.depth_stencil_state_);
    }
    if(d.blend_state_)
    {
        blend_state_ = ctx.render_device->create_blend_state(*d.blend_state_);
    }
    if(d.rasterizer_state_)
    {
        rasterizer_state_ = ctx.render_device->create_rasterizer_state(*d.rasterizer_state_);
    }
}

void PipelinePass::process(PipelinePassDescription const &desc, Pipeline &pipe)
{
    auto const &ctx(pipe.get_context());

    if(desc.recompile_shaders_)
    {
        upload_program(desc, ctx);
        desc.recompile_shaders_ = false;
    }

    if(RenderMode::Custom == rendermode_)
    {
        process_(*this, desc, pipe);
    }
    else
    {
        auto &target = *pipe.current_viewstate().target;

        target.bind(ctx, !writes_only_color_buffer_);
        target.set_viewport(ctx);
        if(depth_stencil_state_)
            ctx.render_context->set_depth_stencil_state(depth_stencil_state_, 1);
        if(blend_state_)
            ctx.render_context->set_blend_state(blend_state_);
        if(rasterizer_state_)
            ctx.render_context->set_rasterizer_state(rasterizer_state_);
        shader_->use(ctx);

        for(auto const &u : desc.uniforms)
        {
            u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
        }

        pipe.bind_gbuffer_input(shader_);
        pipe.bind_light_table(shader_);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / " + name_;
        pipe.begin_gpu_query(ctx, gpu_query_name);
#endif

        if(RenderMode::Callback == rendermode_)
        {
            process_(*this, desc, pipe);
        }
        else
        { // RenderMode::Quad
            pipe.draw_quad();
        }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name);
#endif

        target.unbind(ctx);
        ctx.render_context->reset_state_objects();
    }
}

void PipelinePass::upload_program(PipelinePassDescription const &desc, RenderContext const &ctx)
{
    if(!desc.vertex_shader_.empty() && !desc.fragment_shader_.empty())
    {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
        ResourceFactory factory;
        std::string v_shader = desc.vertex_shader_is_file_name_ ? factory.read_shader_file(desc.vertex_shader_) : desc.vertex_shader_;
        std::string f_shader = desc.fragment_shader_is_file_name_ ? factory.read_shader_file(desc.fragment_shader_) : desc.fragment_shader_;
#else
        std::string v_shader = desc.vertex_shader_is_file_name_ ? Resources::lookup_shader(desc.vertex_shader_) : desc.vertex_shader_;
        std::string f_shader = desc.fragment_shader_is_file_name_ ? Resources::lookup_shader(desc.fragment_shader_) : desc.fragment_shader_;
#endif

        if(!desc.geometry_shader_.empty())
        {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
            std::string g_shader = desc.geometry_shader_is_file_name_ ? factory.read_shader_file(desc.geometry_shader_) : desc.geometry_shader_;
#else
            std::string g_shader = desc.geometry_shader_is_file_name_ ? Resources::lookup_shader(desc.geometry_shader_) : desc.geometry_shader_;
#endif
            shader_->create_from_sources(v_shader, g_shader, f_shader, substitution_map_);
        }
        else
        {
            shader_->create_from_sources(v_shader, f_shader, substitution_map_);
        }

        shader_->upload_to(ctx);
    }
}

} // namespace gua
