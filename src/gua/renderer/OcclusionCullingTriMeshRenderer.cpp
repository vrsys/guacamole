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

#include <queue>

// class header
#include <gua/renderer/OcclusionCullingTriMeshRenderer.hpp>
#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/node/OcclusionCullingGroupNode.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>

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

OcclusionCullingTriMeshRenderer::OcclusionCullingTriMeshRenderer(RenderContext const& ctx, SubstitutionMap const& smap)
    :
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
      VTRenderer(ctx, smap),
#endif
      rs_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK)),
      rs_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE)),
      rs_wireframe_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_BACK)),
      rs_wireframe_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE)),
      default_depth_test_(ctx.render_device->create_depth_stencil_state(true, true,  scm::gl::COMPARISON_LESS)),
      depth_stencil_state_no_test_no_writing_state_(ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER) ),
      default_blend_state_(ctx.render_device->create_blend_state(true)),
      color_accumulation_state_(ctx.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD)), 
      default_rendering_program_stages_(), default_rendering_programs_(),
      depth_complexity_vis_program_stages_(), depth_complexity_vis_program_(nullptr), 
      global_substitution_map_(smap)
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_depth_complexity_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader_no_programmable_material.vert");
    std::string f_depth_complexity_vis = factory.read_shader_file("resources/shaders/depth_complexity_to_color.frag");
    
    std::string v_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
    std::string f_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");

    //std::string v_occl = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
    //std::string f_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");
#else
    std::string v_depth_complexity_vis = Resources::lookup_shader("shaders/tri_mesh_shader_no_programmable_material.vert");
    std::string f_depth_complexity_vis = Resources::lookup_shader("shaders/depth_complexity_to_color.frag");

    std::string v_default_rendering_vis = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
    std::string f_default_rendering_vis = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
#endif

    depth_complexity_vis_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_depth_complexity_vis);
    depth_complexity_vis_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_depth_complexity_vis);

    default_rendering_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_default_rendering_vis);
    default_rendering_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_default_rendering_vis);
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
        
    switch(desc.get_occlusion_culling_mode()) {
        case OcclusionCullingMode::No_Culling: {
            render_without_oc(pipe, desc);
            break;
        }

        case OcclusionCullingMode::Naive_Stop_And_Wait: {
            render_naive_stop_and_wait_oc(pipe, desc);
            break;
        }

        case OcclusionCullingMode::Hierarchical_Stop_And_Wait: {
            render_hierarchical_stop_and_wait_oc(pipe, desc);
            break;
        }

        case OcclusionCullingMode::Coherent_Hierarchical_Culling: {
            
            std::cout << "CHC not implemented" << std::endl;
            //render_CHC_base(desc, desc);
            break;
        }

        default: {
            std::cout << "Unknown occlusion culling mode" << std::endl;
        }

    }
}


void OcclusionCullingTriMeshRenderer::render_without_oc(Pipeline& pipe, PipelinePassDescription const& desc) {
   
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
            target.set_viewport(ctx);

            scm::math::vec2ui render_target_dims(target.get_width(), target.get_height());


            int view_id(camera.config.get_view_id());

            MaterialShader* current_material(nullptr);
            std::shared_ptr<ShaderProgram> current_shader;
            auto current_rasterizer_state = rs_cull_back_;
            ctx.render_context->apply();

            // loop through all objects, sorted by material ----------------------------
            //std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl; 

            
        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();


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

            if(!depth_complexity_vis) {
                switch_state_based_on_node_material(ctx, tri_mesh_node, current_shader, current_material, target, 
                                                    pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);
            } else {
                switch_state_for_depth_complexity_vis(ctx, current_shader);

            }

            if(current_shader && tri_mesh_node->get_geometry())
            {
                upload_uniforms_for_node(ctx, tri_mesh_node, current_shader, pipe, current_rasterizer_state);
                
                tri_mesh_node->get_geometry()->draw(pipe.get_context());
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


void OcclusionCullingTriMeshRenderer::render_naive_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc) {
   
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
            target.set_viewport(ctx);

            scm::math::vec2ui render_target_dims(target.get_width(), target.get_height());


            int view_id(camera.config.get_view_id());

            MaterialShader* current_material(nullptr);
            std::shared_ptr<ShaderProgram> current_shader;
            auto current_rasterizer_state = rs_cull_back_;
            ctx.render_context->apply();

            // loop through all objects, sorted by material ----------------------------
            //std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl; 

            
        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();


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

            if(!depth_complexity_vis) {
                switch_state_based_on_node_material(ctx, tri_mesh_node, current_shader, current_material, target, 
                                                    pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);
            } else {
                switch_state_for_depth_complexity_vis(ctx, current_shader);

            }

            if(current_shader && tri_mesh_node->get_geometry())
            {
                upload_uniforms_for_node(ctx, tri_mesh_node, current_shader, pipe, current_rasterizer_state);
                
                tri_mesh_node->get_geometry()->draw(pipe.get_context());
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

void OcclusionCullingTriMeshRenderer::render_hierarchical_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc) {
    RenderContext const& ctx(pipe.get_context());

    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));


    if(sorted_occlusion_group_nodes != scene.nodes.end() && !sorted_occlusion_group_nodes->second.empty())
    {
        auto& target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";
        std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";

        pipe.begin_gpu_query(ctx, gpu_query_name);
        pipe.begin_cpu_query(cpu_query_name);
#endif

        bool write_depth = true;
        target.bind(ctx, write_depth);
        target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(target.get_width(), target.get_height());


        int view_id(camera.config.get_view_id());

        MaterialShader* current_material(nullptr);
        std::shared_ptr<ShaderProgram> current_shader;
        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();

        // loop through all objects, sorted by material ----------------------------
        std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl; 


        // get a (serialized) cam node (see: guacamole/src/gua/node/CameraNode.cpp and guacamole/src/gua/node/CameraNode.hpp)
        auto const& current_cam_node = pipe.current_viewstate().camera;
        std::cout << "Current Cam UUID: " << current_cam_node.uuid << std::endl;

        // reset visibility status if the node was never visited
        for( auto& occlusion_group_node : sorted_occlusion_group_nodes->second ) {

            // if we never checked set the visibility status for this node, it will be 0.
            // in this case, we recursively set the entire hierarchy to visible

            int32_t frame_id = occlusion_group_node->get_last_visibility_check_frame_id(current_cam_node.uuid);

            std::cout << "FRAME ID: " << frame_id << std::endl;

            if( 0 != frame_id) {
                continue;
            }



            std::cout << "Setting hierarchy to visible" << std::endl;
            // use a queue for breadth first search (stack would do breadth first search)
            std::queue<gua::node::Node*> traversal_queue;

            // add root node of our occlusion hierarchy to the traversal queue 
            traversal_queue.push(occlusion_group_node);

            while(!traversal_queue.empty()) {
                // get next node
                gua::node::Node* current_node = traversal_queue.front();
                // work on it (CHC-Style)
                current_node->set_visibility(current_cam_node.uuid, true);

                //remove this node from the queue
                traversal_queue.pop();

                //push all children (currently in arbitrary order)
                for(std::shared_ptr<gua::node::Node> const& shared_child_node_ptr : current_node->get_children()) {

                    // the vector returned by "get_children" unfortunately contains shared_ptrs instead of raw ptrs.
                    // the serializer however creates raw prts. Calling ".get()" on a shared ptr provides us with the
                    // raw ptr that is referenced by the manager object 
                    gua::node::Node* raw_child_node_ptr = shared_child_node_ptr.get();
                    traversal_queue.push(raw_child_node_ptr);
                }
            }
        }


        //each of the occlusion group nodes models an entire object hierarchy
        for(auto const& occlusion_group_node : sorted_occlusion_group_nodes->second) {

            //if(occlusion_group_node->)

            // use a queue for breadth first search (stack would do breadth first search)
            std::queue<gua::node::Node*> traversal_queue;

            // add root node of our occlusion hierarchy to the traversal queue 
            traversal_queue.push(occlusion_group_node);


            while(!traversal_queue.empty()) {
                // get next node
                gua::node::Node* current_node = traversal_queue.front();
                // work on it (CHC-Style)
                std::cout << "Visited node: " << current_node->get_name() <<  "\t\t Visibility Status: " << current_node->get_visibility(current_cam_node.uuid) << std::endl; 


                //remove this node from the queue
                traversal_queue.pop();

                //push all children (currently in arbitrary order)
                for(std::shared_ptr<gua::node::Node> const& shared_child_node_ptr : current_node->get_children()) {

                    // the vector returned by "get_children" unfortunately contains shared_ptrs instead of raw ptrs.
                    // the serializer however creates raw prts. Calling ".get()" on a shared ptr provides us with the
                    // raw ptr that is referenced by the manager object 
                    gua::node::Node* raw_child_node_ptr = shared_child_node_ptr.get();
                    traversal_queue.push(raw_child_node_ptr);
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


////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::upload_uniforms_for_node(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader, 
                                                               Pipeline& pipe, scm::gl::rasterizer_state_ptr& current_rasterizer_state) {

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

    ctx.render_context->apply_program();

}


////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::switch_state_based_on_node_material(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader, 
                                                                          MaterialShader* current_material, RenderTarget const& target, bool shadow_mode, std::size_t cam_uuid) {
            if(current_material != tri_mesh_node->get_material()->get_shader())
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
                    current_shader->set_uniform(ctx, math::vec2ui(target.get_width(), target.get_height()),
                                                "gua_resolution"); // TODO: pass gua_resolution. Probably should be somehow else implemented
                    current_shader->set_uniform(ctx, 1.0f / target.get_width(), "gua_texel_width");
                    current_shader->set_uniform(ctx, 1.0f / target.get_height(), "gua_texel_height");
                    // hack
                    current_shader->set_uniform(ctx, ::get_handle(target.get_depth_buffer()), "gua_gbuffer_depth");


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
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::switch_state_for_depth_complexity_vis(RenderContext const& ctx, std::shared_ptr<ShaderProgram>& current_shader) {
    if(nullptr == depth_complexity_vis_program_) {
        depth_complexity_vis_program_ = std::make_shared<ShaderProgram>();
        depth_complexity_vis_program_->set_shaders(depth_complexity_vis_program_stages_, std::list<std::string>(), false, global_substitution_map_);
    }

    if(current_shader != depth_complexity_vis_program_) {
        current_shader = depth_complexity_vis_program_;
        current_shader->use(ctx);
    }


    if(    ctx.render_context->current_blend_state() != color_accumulation_state_
        || ctx.render_context->current_depth_stencil_state() != depth_stencil_state_no_test_no_writing_state_)
    {
        ctx.render_context->set_blend_state(color_accumulation_state_);
        ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_test_no_writing_state_);
        ctx.render_context->apply_state_objects();
    }
}



////////////////////////////////////////////////////////////////////////////////

} // namespace gua
