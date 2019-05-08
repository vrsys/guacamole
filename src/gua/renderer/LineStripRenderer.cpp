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
#include <gua/renderer/LineStripRenderer.hpp>

#include <gua/config.hpp>
#include <gua/node/LineStripNode.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/LineStripResource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

#include <scm/core/math/math.h>

namespace
{
gua::math::vec2ui get_handle(scm::gl::texture_image_ptr const& tex)
{
    uint64_t handle = 0;
    if(tex)
    {
        handle = tex->native_handle();
    }
    return gua::math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

} // namespace

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

LineStripRenderer::LineStripRenderer(RenderContext const& ctx, SubstitutionMap const& smap)
    : rs_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK)),
      rs_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE)),
      rs_wireframe_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_BACK)),
      rs_wireframe_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE)), program_stages_(), programs_(), volumetric_point_programs_(),
      volumetric_line_programs_(), global_substitution_map_(smap)
{
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif

#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("resources/shaders/point_line_strip_shader_non_volumetric.vert");
    std::string f_shader = factory.read_shader_file("resources/shaders/point_line_strip_shader_non_volumetric.frag");

    std::string volumetric_point_line_v_shader = factory.read_shader_file("resources/shaders/point_line_strip_shader_volumetric.vert");
    std::string volumetric_point_g_shader = factory.read_shader_file("resources/shaders/point_shader_volumetric.geom");
    std::string volumetric_line_g_shader = factory.read_shader_file("resources/shaders/line_strip_shader_volumetric.geom");
    std::string volumetric_point_line_f_shader = factory.read_shader_file("resources/shaders/point_line_strip_shader_volumetric.frag");

    // std::string volumetric__v_shader = factory.read_shader_file("resources/shaders/point_line_strip_shader_volumetric.vert");
    // std::string volumetric_point_g_shader = factory.read_shader_file("resources/shaders/line_strip_shader_volumetric.geom");
    // std::string volumetric_point_f_shader = factory.read_shader_file("resources/shaders/point_line_strip_shader_volumetric.frag");
#endif
    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, v_shader));
    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));

    volumetric_point_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, volumetric_point_line_v_shader));
    volumetric_point_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, volumetric_point_g_shader));
    volumetric_point_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, volumetric_point_line_f_shader));

    volumetric_line_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, volumetric_point_line_v_shader));
    volumetric_line_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, volumetric_line_g_shader));
    volumetric_line_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, volumetric_point_line_f_shader));
}

////////////////////////////////////////////////////////////////////////////////

void LineStripRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::LineStripNode))));

    if(sorted_objects != scene.nodes.end() && sorted_objects->second.size() > 0)
    {
        auto& target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

        std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
            return reinterpret_cast<node::LineStripNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::LineStripNode*>(b)->get_material()->get_shader();
        });

        RenderContext const& ctx(pipe.get_context());

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";
        std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";

        pipe.begin_gpu_query(ctx, gpu_query_name);
        pipe.begin_cpu_query(cpu_query_name);
#endif

        bool write_depth = true;
        target.bind(ctx, write_depth);
        target.set_viewport(ctx);

        int view_id(camera.config.get_view_id());

        MaterialShader* current_material_shader(nullptr);
        std::shared_ptr<ShaderProgram> current_shader_program;
        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();

        // loop through all objects, sorted by material ----------------------------
        for(auto const& object : sorted_objects->second)
        {
            auto line_strip_node(reinterpret_cast<node::LineStripNode*>(object));
            if(pipe.current_viewstate().shadow_mode && line_strip_node->get_shadow_mode() == ShadowMode::OFF)
            {
                continue;
            }

            if(!line_strip_node->get_render_to_gbuffer())
            {
                continue;
            }

            std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>>* current_material_shader_map = nullptr;

            std::vector<ShaderProgramStage>* current_shader_stages = nullptr;

            // select the material shader maps belonging to the current visualization mode:
            // non volumetric line strips and points share the same shader
            if(!line_strip_node->get_render_volumetric())
            {
                current_material_shader_map = &programs_;
                current_shader_stages = &program_stages_;
            }
            else
            {
                // volumetric point rendering
                if(line_strip_node->get_render_vertices_as_points())
                {
                    current_material_shader_map = &volumetric_point_programs_;
                    current_shader_stages = &volumetric_point_program_stages_;
                }
                else
                { // volumetric
                    current_material_shader_map = &volumetric_line_programs_;
                    current_shader_stages = &volumetric_line_program_stages_;
                }
            }

            if(current_material_shader != line_strip_node->get_material()->get_shader())
            {
                current_material_shader = line_strip_node->get_material()->get_shader();
                if(current_material_shader)
                {
                    auto shader_iterator = current_material_shader_map->find(current_material_shader);
                    if(shader_iterator != current_material_shader_map->end())
                    {
                        current_shader_program = shader_iterator->second;
                    }
                    else
                    {
                        auto smap = global_substitution_map_;
                        for(const auto& i : current_material_shader->generate_substitution_map())
                            smap[i.first] = i.second;

                        current_shader_program = std::make_shared<ShaderProgram>();
                        current_shader_program->set_shaders(*current_shader_stages, std::list<std::string>(), false, smap);
                        (*current_material_shader_map)[current_material_shader] = current_shader_program;
                    }
                }
                else
                {
                    Logger::LOG_WARNING << "LineStripPass::process(): Cannot find material: " << line_strip_node->get_material()->get_shader_name() << std::endl;
                }
                //}

                if(current_shader_program)
                {
                    current_shader_program->use(ctx);
                    current_shader_program->set_uniform(ctx,
                                                        math::vec2ui(target.get_width(), target.get_height()),
                                                        "gua_resolution"); // TODO: pass gua_resolution. Probably should be somehow else implemented
                    current_shader_program->set_uniform(ctx, 1.0f / target.get_width(), "gua_texel_width");
                    current_shader_program->set_uniform(ctx, 1.0f / target.get_height(), "gua_texel_height");
                    // hack
                    current_shader_program->set_uniform(ctx, ::get_handle(target.get_depth_buffer()), "gua_gbuffer_depth");
                }
            }

            if(current_shader_program && line_strip_node->get_geometry())
            {
                auto model_view_mat = scene.rendering_frustum.get_view() * line_strip_node->get_cached_world_transform();
                UniformValue normal_mat(math::mat4f(scm::math::transpose(scm::math::inverse(line_strip_node->get_cached_world_transform()))));

                int rendering_mode = pipe.current_viewstate().shadow_mode ? (line_strip_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

                current_shader_program->apply_uniform(ctx, "gua_model_matrix", math::mat4f(line_strip_node->get_cached_world_transform()));
                current_shader_program->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
                current_shader_program->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
                current_shader_program->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);

                // lowfi shadows dont need material input
                if(rendering_mode != 1)
                {
                    line_strip_node->get_material()->apply_uniforms(ctx, current_shader_program.get(), view_id);
                }

                bool show_backfaces = line_strip_node->get_material()->get_show_back_faces();
                bool render_wireframe = line_strip_node->get_material()->get_render_wireframe();

                if(show_backfaces)
                {
                    if(render_wireframe)
                    {
                        current_rasterizer_state = rs_wireframe_cull_none_;
                    }
                    else
                    {
                        current_rasterizer_state = rs_cull_none_;
                    }
                }
                else
                {
                    if(render_wireframe)
                    {
                        current_rasterizer_state = rs_wireframe_cull_back_;
                    }
                    else
                    {
                        current_rasterizer_state = rs_cull_back_;
                    }
                }

                float screen_space_line_width = line_strip_node->get_screen_space_line_width();
                // float screen_space_point_size = line_strip_node->get_screen_space_point_size();

                if(ctx.render_context->current_rasterizer_state() != current_rasterizer_state)
                {
                    ctx.render_context->set_rasterizer_state(current_rasterizer_state, screen_space_line_width);
                    ctx.render_context->apply_state_objects();
                }

                ctx.render_context->apply_program();

                line_strip_node->get_geometry()->draw(pipe.get_context(), line_strip_node->get_render_vertices_as_points(), line_strip_node->get_render_lines_as_strip());
            }
        }

        target.unbind(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name);
        pipe.end_cpu_query(cpu_query_name);
#endif

        ctx.render_context->reset_state_objects();
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
