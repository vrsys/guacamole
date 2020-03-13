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
#include <gua/renderer/TriMeshRenderer.hpp>

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/WindowDatabase.hpp>
#include <vector>

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

TriMeshRenderer::TriMeshRenderer(RenderContext const& ctx, SubstitutionMap const& smap)
    :
        OcclusionCullingAwareRenderer(ctx, smap),
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
      VTRenderer(ctx, smap),
#endif
      rs_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK)),
      rs_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE)),
      rs_wireframe_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_BACK)),
      rs_wireframe_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE)), program_stages_(), programs_(), global_substitution_map_(smap)
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
    std::string f_shader = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");
#else
    std::string v_shader = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
    std::string f_shader = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
#endif

    program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_shader);
    program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_shader);
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{


    RenderContext const& ctx(pipe.get_context());

    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::TriMeshNode))));



    if(sorted_objects != scene.nodes.end() && !sorted_objects->second.empty())
    {
        auto& target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

        std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
            return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
        });

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";
        std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";

        pipe.begin_gpu_query(ctx, gpu_query_name);
        pipe.begin_cpu_query(cpu_query_name);
#endif

        bool write_depth = true;
        target.bind(ctx, write_depth);

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        target.set_side_by_side_viewport_array(ctx);

#else
        target.set_viewport(ctx);

#endif //GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING

        int view_id(camera.config.get_view_id());

        MaterialShader* current_material(nullptr);
        std::shared_ptr<ShaderProgram> current_shader;
        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();

        bool is_instanced_side_by_side_enabled = false;

#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
        auto associated_window = gua::WindowDatabase::instance()->lookup(camera.config.output_window_name());//->add left_output_window
        
        if(associated_window->config.get_stereo_mode() == StereoMode::SIDE_BY_SIDE) {
            is_instanced_side_by_side_enabled = true;
        }
#endif

        // loop through all objects, sorted by material ----------------------------
        for(auto const& object : sorted_objects->second)
        {
            auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));
            if(pipe.current_viewstate().shadow_mode && tri_mesh_node->get_shadow_mode() == ShadowMode::OFF)
            {
                continue;
            }

            if(!tri_mesh_node->get_render_to_gbuffer())
            {
                continue;
            }

            if(current_material != tri_mesh_node->get_material()->get_shader())
            {
                current_material = tri_mesh_node->get_material()->get_shader();
                if(current_material)
                {
                    auto shader_iterator = programs_.find(current_material);
                    if(shader_iterator != programs_.end())
                    {
                        current_shader = shader_iterator->second;
                    }
                    else
                    {
                        auto smap = global_substitution_map_;
                        for(const auto& i : current_material->generate_substitution_map())
                            smap[i.first] = i.second;

                        current_shader = std::make_shared<ShaderProgram>();

#ifndef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
                        current_shader->set_shaders(program_stages_, std::list<std::string>(), false, smap);
#else
                        bool virtual_texturing_enabled = !pipe.current_viewstate().shadow_mode && tri_mesh_node->get_material()->get_enable_virtual_texturing();
                        current_shader->set_shaders(program_stages_, std::list<std::string>(), false, smap, virtual_texturing_enabled);
#endif
                        programs_[current_material] = current_shader;
                    }
                }
                else
                {
                    Logger::LOG_WARNING << "TriMeshPass::process(): Cannot find material: " << tri_mesh_node->get_material()->get_shader_name() << std::endl;
                }
                if(current_shader)
                {
                    current_shader->use(ctx);
                    current_shader->set_uniform(ctx, math::vec2ui(target.get_width(), target.get_height()),
                                                "gua_resolution"); // TODO: pass gua_resolution. Probably should be somehow else implemented
                    current_shader->set_uniform(ctx, 1.0f / target.get_width(), "gua_texel_width");
                    current_shader->set_uniform(ctx, 1.0f / target.get_height(), "gua_texel_height");
                    // hack
                    current_shader->set_uniform(ctx, ::get_handle(target.get_depth_buffer()), "gua_gbuffer_depth");

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
                    if(!pipe.current_viewstate().shadow_mode)
                    {
                        VTContextState* vt_state = &VTBackend::get_instance().get_state(pipe.current_viewstate().camera.uuid);

                        if(vt_state && vt_state->has_camera_)
                        {
                            current_shader->set_uniform(ctx, vt_state->feedback_enabled_, "enable_feedback");
                        }
                    }
#endif
                }
            }

            if(current_shader && tri_mesh_node->get_geometry())
            {
                auto const node_world_transform = tri_mesh_node->get_latest_cached_world_transform(ctx.render_window);

				auto model_view_mat = scene.rendering_frustum.get_view() * node_world_transform;
                UniformValue normal_mat(math::mat4f(scm::math::transpose(scm::math::inverse(node_world_transform))));

                int rendering_mode = pipe.current_viewstate().shadow_mode ? (tri_mesh_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

                current_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(node_world_transform));
                current_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
                current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
                current_shader->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);
#ifdef GUACAMOLE_ENABLE_MULTI_VIEW_RENDERING
                if(is_instanced_side_by_side_enabled) {
                    auto secondary_model_view_mat = scene.secondary_rendering_frustum.get_view() * node_world_transform;
                    current_shader->apply_uniform(ctx, "gua_secondary_model_view_matrix", math::mat4f(secondary_model_view_mat));
                }
#endif

                // lowfi shadows dont need material input
                if(rendering_mode != 1)
                {
                    tri_mesh_node->get_material()->apply_uniforms(ctx, current_shader.get(), view_id);
                }

                bool show_backfaces = tri_mesh_node->get_material()->get_show_back_faces();
                bool render_wireframe = tri_mesh_node->get_material()->get_render_wireframe();

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

                if(ctx.render_context->current_rasterizer_state() != current_rasterizer_state)
                {
                    ctx.render_context->set_rasterizer_state(current_rasterizer_state);
                    ctx.render_context->apply_state_objects();
                }


                current_rasterizer_state = rs_cull_none_;
                ctx.render_context->apply_program();

                if(is_instanced_side_by_side_enabled) {
                    tri_mesh_node->get_geometry()->draw_instanced(pipe.get_context(), 2);
                } else {

                    tri_mesh_node->get_geometry()->draw(pipe.get_context());                   
                }

            }
        }




        target.unbind(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name);
        pipe.end_cpu_query(cpu_query_name);
#endif
        ctx.render_context->reset_state_objects();
        ctx.render_context->sync();
    }
}


void TriMeshRenderer::renderSingleNode(Pipeline& pipe, PipelinePassDescription const& desc, gua::node::Node* const current_node, RenderInfo& current_render_info ) {



//could still be optimized by setting more things in the
    RenderContext const& ctx(pipe.get_context());
    MaterialShader* current_material = current_render_info.material;
    std::shared_ptr<ShaderProgram> current_shader = current_render_info.shader;
    auto current_rasterizer_state = current_render_info.rasterizer_state;



    ctx.render_context->apply(); 
    if(std::type_index(typeid(node::TriMeshNode)) == std::type_index(typeid(*current_node)) ) {
        auto& render_target = *pipe.current_viewstate().target;


        auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(current_node));

                    if (depth_complexity_visualization_) {
                current_shader = depth_complexity_vis_program_;
                current_shader->use(ctx);

                ctx.render_context->set_blend_state(color_accumulation_state_);
                ctx.render_context->set_depth_stencil_state(depth_stencil_state_writing_without_test_state_);
                ctx.render_context->apply_state_objects();
            }  
        else if                                                                                                                                                                                                                                                                                                                                                                                                                             (current_material != tri_mesh_node->get_material()->get_shader())
        {
            current_material = tri_mesh_node->get_material()->get_shader();
            if(current_material)
            {
                auto shader_iterator = default_rendering_programs_.find(current_material);
                if(shader_iterator != default_rendering_programs_.end())
                {
                    current_shader = shader_iterator->second;
                }
                else
                {
                    auto smap = global_substitution_map_;
                    for(const auto& i : current_material->generate_substitution_map())
                        smap[i.first] = i.second;

                    current_shader = std::make_shared<ShaderProgram>();

    #ifndef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
                    current_shader->set_shaders(default_rendering_program_stages_, std::list<std::string>(), false, smap);
    #else
                    bool virtual_texturing_enabled = !shadow_mode && tri_mesh_node->get_material()->get_enable_virtual_texturing();
                    current_shader->set_shaders(default_rendering_program_stages_, std::list<std::string>(), false, smap, virtual_texturing_enabled);
    #endif
                    default_rendering_programs_[current_material] = current_shader;
                }
            }
            else
            {
                Logger::LOG_WARNING << "OcclusionCullingTriMeshPass::process(): Cannot find material: " << tri_mesh_node->get_material()->get_shader_name() << std::endl;
            }
            if(current_shader)
            {
                current_shader->use(ctx);
                current_shader->set_uniform(ctx, math::vec2ui(render_target.get_width(), render_target.get_height()),
                                            "gua_resolution"); // TODO: pass gua_resolution. Probably should be somehow else implemented
                current_shader->set_uniform(ctx, 1.0f / render_target.get_width(), "gua_texel_width");
                current_shader->set_uniform(ctx, 1.0f / render_target.get_height(), "gua_texel_height");
                // hack
                current_shader->set_uniform(ctx, ::get_handle(render_target.get_depth_buffer()), "gua_gbuffer_depth");


    #ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
                if(!shadow_mode)
                {
                    VTContextState* vt_state = &VTBackend::get_instance().get_state(cam_uuid);

                    if(vt_state && vt_state->has_camera_)
                    {
                        current_shader->set_uniform(ctx, vt_state->feedback_enabled_, "enable_feedback");
                    }
                }
    #endif
            }
        }


        if(current_shader && tri_mesh_node->get_geometry())
        {

            auto& scene = *pipe.current_viewstate().scene;
            auto const node_world_transform = tri_mesh_node->get_latest_cached_world_transform(ctx.render_window);


            auto model_view_mat = scene.rendering_frustum.get_view() * node_world_transform;
            UniformValue normal_mat(math::mat4f(scm::math::transpose(scm::math::inverse(node_world_transform))));

            int rendering_mode = pipe.current_viewstate().shadow_mode ? (tri_mesh_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

            current_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(node_world_transform));
            current_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
            current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
            current_shader->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);

            // lowfi shadows dont need material input
            if(rendering_mode != 1)
            {
                auto view_id = pipe.current_viewstate().camera.config.get_view_id();
                tri_mesh_node->get_material()->apply_uniforms(ctx, current_shader.get(), view_id);
            }

            bool show_backfaces = tri_mesh_node->get_material()->get_show_back_faces();
            bool render_wireframe = tri_mesh_node->get_material()->get_render_wireframe();


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

            if(ctx.render_context->current_rasterizer_state() != current_rasterizer_state)
            {
                ctx.render_context->set_rasterizer_state(current_rasterizer_state);
                ctx.render_context->apply_state_objects();
            }
            
            ctx.render_context->apply_program();
            
            tri_mesh_node->get_geometry()->draw(pipe.get_context());
        }
        

    }

}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
