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
#include <gua/databases/WindowDatabase.hpp>

namespace gua
{

////////////////////////////////////////////////////////////////////////////////
void PipelinePassDescription::touch() { ++mod_count_; }

////////////////////////////////////////////////////////////////////////////////
void PipelinePassDescription::enable(bool enable) {
    private_.is_enabled_ = enable;
}

////////////////////////////////////////////////////////////////////////////////
bool PipelinePassDescription::is_enabled() const {
    return private_.is_enabled_;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& PipelinePassDescription::name() const { return private_.name_; }

////////////////////////////////////////////////////////////////////////////////
unsigned PipelinePassDescription::mod_count() const { return mod_count_; }

const std::vector<std::shared_ptr<PipelineResponsibilityDescription>>& PipelinePassDescription::get_responsibilities() const { return pipeline_responsibilities_; }

bool PipelinePass::needs_color_buffer_as_input() const { return private_.needs_color_buffer_as_input_; }
bool PipelinePass::writes_only_color_buffer() const { return private_.writes_only_color_buffer_; }
bool PipelinePass::enable_for_shadows() const { return private_.enable_for_shadows_; }

scm::gl::rasterizer_state_ptr PipelinePass::rasterizer_state() const { return rasterizer_state_; }
scm::gl::depth_stencil_state_ptr PipelinePass::depth_stencil_state() const { return depth_stencil_state_; }
scm::gl::blend_state_ptr PipelinePass::blend_state() const { return blend_state_; }
std::shared_ptr<ShaderProgram> PipelinePass::shader() const { return shader_; }

bool operator==(PipelineResponsibilityDescription const& lhs, PipelineResponsibilityDescription const& rhs) {
    return lhs.private_.name_ == rhs.private_.name_;
}
bool operator!=(PipelineResponsibilityDescription const& lhs, PipelineResponsibilityDescription const& rhs) { return lhs.private_.name_ != rhs.private_.name_; }

////////////////////////////////////////////////////////////////////////////////
PipelinePass::PipelinePass(PipelinePassDescription const& d, RenderContext const& ctx, SubstitutionMap const& substitution_map) : private_(d.private_), substitution_map_(substitution_map)
{
    shader_ = std::make_shared<ShaderProgram>();

    upload_program(d, ctx);

    if(d.private_.depth_stencil_state_desc_)
    {
        depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(*d.private_.depth_stencil_state_desc_);
    }
    if(d.private_.blend_state_desc_)
    {
        blend_state_ = ctx.render_device->create_blend_state(*d.private_.blend_state_desc_);
    }
    if(d.private_.rasterizer_state_desc_)
    {
        rasterizer_state_ = ctx.render_device->create_rasterizer_state(*d.private_.rasterizer_state_desc_);
    }
}
void PipelinePass::process(PipelinePassDescription const& desc, Pipeline& pipe, bool render_multiview, bool use_hardware_mvr)
{
    if(private_.is_enabled_) {
        auto const& ctx(pipe.get_context());

        if(desc.recompile_shaders_)
        {
            upload_program(desc, ctx);
            desc.recompile_shaders_ = false;
        }

        if(RenderMode::Custom == private_.rendermode_)
        {
            private_.process_(*this, desc, pipe, render_multiview, use_hardware_mvr);
        }
        else
        {
            auto& target = *pipe.current_viewstate().target;

            target.bind(ctx, !private_.writes_only_color_buffer_);
            target.set_viewport(ctx);
            if(depth_stencil_state_)
                ctx.render_context->set_depth_stencil_state(depth_stencil_state_, 1);
            if(blend_state_)
                ctx.render_context->set_blend_state(blend_state_);
            if(rasterizer_state_)
                ctx.render_context->set_rasterizer_state(rasterizer_state_);
            shader_->use(ctx);

            for(auto const& u : desc.uniforms)
            {
                u.second.apply(ctx, u.first, ctx.render_context->current_program(), 0);
            }

            pipe.bind_gbuffer_input(shader_);
            pipe.bind_light_table(shader_);

    #ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
            std::string gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / " + private_->name_;
            pipe.begin_gpu_query(ctx, gpu_query_name);
    #endif

            if(RenderMode::Callback == private_.rendermode_)
            {
                private_.process_(*this, desc, pipe, render_multiview, use_hardware_mvr);
            }
            else
            { // RenderMode::Quad
                if(render_multiview /*& !use_hardware_mvr*/) {
                    pipe.draw_quad_instanced();
                    //std::cout << "Drawing quad instanced" << std::endl;
                } else {
                    pipe.draw_quad();
                    //std::cout << "Drawing something not instanced" << std::endl;
                }
            }

    #ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
            pipe.end_gpu_query(ctx, gpu_query_name);
    #endif

            target.unbind(ctx);
            ctx.render_context->reset_state_objects();
        }

    }
}

void PipelinePass::upload_program(PipelinePassDescription const& desc, RenderContext const& ctx)
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

PipelineResponsibility::PipelineResponsibility(PipelineResponsibilityDescription const& d, Pipeline& pipe) : private_(d.private_) {}
} // namespace gua
