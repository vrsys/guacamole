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


#include <scm/gl_core/render_device/opengl/gl_core.h>
//#define OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE

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
      rs_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK)),   // if backface culling is enabled
      rs_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE)),   // if backface culling is disabled
      rs_wireframe_cull_back_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_BACK)),  //if backface culling is enabled and the object is supposed to be rendered as wireframe
      rs_wireframe_cull_none_(ctx.render_device->create_rasterizer_state(scm::gl::FILL_WIREFRAME, scm::gl::CULL_NONE)),  //if backface culling is enabled and the object is supposed to be rendered as wireframe
      default_depth_test_(ctx.render_device->create_depth_stencil_state(true, true,  scm::gl::COMPARISON_LESS)), /* < for rendering > */
      depth_stencil_state_no_test_no_writing_state_(ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER) ),
      depth_stencil_state_writing_without_test_state_(ctx.render_device->create_depth_stencil_state(false, true, scm::gl::COMPARISON_LESS) ),
      depth_stencil_state_test_without_writing_state_(ctx.render_device->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS) ), /* < for occlusion querying > */

      default_blend_state_(ctx.render_device->create_blend_state(false)),  /* < for rendering > */
      color_accumulation_state_(ctx.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD)),  

      default_rendering_program_stages_(), default_rendering_programs_(), //a map that stores as many shaders as nodes with different material are encountered. The material input is substituted
      depth_complexity_vis_program_stages_(), depth_complexity_vis_program_(nullptr),//only one shader that is independent of the actual node material
      occlusion_query_box_program_stages_(), occlusion_query_box_program_(nullptr), //only one shader that is independent of the actual node material
      global_substitution_map_(smap)
{

// define all our shader sources for our 3 different shader programs we need/want to use here
// 1
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_occlusion_query_box = factory.read_shader_file("resources/shaders/occlusion_query_box.vert");
    std::string f_occlusion_query_box = factory.read_shader_file("resources/shaders/occlusion_query_box.frag");

    std::string v_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
    std::string f_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");
                                                          
    std::string v_depth_complexity_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader_no_programmable_material.vert");
    std::string f_depth_complexity_vis = factory.read_shader_file("resources/shaders/depth_complexity_to_color.frag");
#else
    std::string v_occlusion_query_box = Resources::lookup_shader("shaders/occlusion_query_box.vert");
    std::string f_occlusion_query_box = Resources::lookup_shader("shaders/occlusion_query_box.frag");

    std::string v_default_rendering_vis = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
    std::string f_default_rendering_vis = Resources::lookup_shader("shaders/tri_mesh_shader.frag");

    std::string v_depth_complexity_vis = Resources::lookup_shader("shaders/tri_mesh_shader_no_programmable_material.vert");
    std::string f_depth_complexity_vis = Resources::lookup_shader("shaders/depth_complexity_to_color.frag");
#endif

    default_rendering_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_default_rendering_vis);
    default_rendering_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_default_rendering_vis);

    occlusion_query_box_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_occlusion_query_box);
    occlusion_query_box_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_occlusion_query_box);

    depth_complexity_vis_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_depth_complexity_vis);
    depth_complexity_vis_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_depth_complexity_vis);
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{

    if(nullptr == occlusion_query_box_program_) {
        occlusion_query_box_program_ = std::make_shared<ShaderProgram>();
        occlusion_query_box_program_->set_shaders(occlusion_query_box_program_stages_, std::list<std::string>(), false, global_substitution_map_);
    }

    if(nullptr == depth_complexity_vis_program_) {
        depth_complexity_vis_program_ = std::make_shared<ShaderProgram>();
        depth_complexity_vis_program_->set_shaders(depth_complexity_vis_program_stages_, std::list<std::string>(), false, global_substitution_map_);
    }


    auto const& frustum = pipe.current_viewstate().frustum;
    scm::math::mat4d const view_matrix = frustum.get_view();
    scm::math::mat4d const projection_matrix = frustum.get_projection();
    scm::math::mat4d view_projection_matrix = projection_matrix * view_matrix;


    scm::math::mat4d const camera_matrix = scm::math::inverse(view_matrix);

    gua::math::vec4f camera_space_cam_pos_homogeneous(0.0f, 0.0f, 0.0f, 1.0f);
    gua::math::vec4f world_space_cam_pos_homogeneous = gua::math::mat4f(camera_matrix) * camera_space_cam_pos_homogeneous;

    gua::math::vec3f world_space_cam_pos_euclidean(world_space_cam_pos_homogeneous[0], world_space_cam_pos_homogeneous[1], world_space_cam_pos_homogeneous[2]);


    //std::cout << "World space cam pos: " << world_space_cam_pos_euclidean << std::endl;


    switch(desc.get_occlusion_culling_strategy()) {
        case OcclusionCullingStrategy::No_Culling: {
            render_without_oc(pipe, desc, view_projection_matrix, world_space_cam_pos_euclidean);
            break;
        }

        case OcclusionCullingStrategy::Naive_Stop_And_Wait: {
            render_naive_stop_and_wait_oc(pipe, desc, view_projection_matrix, world_space_cam_pos_euclidean);
            break;
        }

        case OcclusionCullingStrategy::Hierarchical_Stop_And_Wait: {
            render_hierarchical_stop_and_wait_oc(pipe, desc, view_projection_matrix, world_space_cam_pos_euclidean);
            break;
        }

        case OcclusionCullingStrategy::Coherent_Hierarchical_Culling: {
            
            std::cout << "CHC not implemented" << std::endl;
            //render_CHC_base(desc, desc);
            break;
        }

        default: {
            std::cout << "Unknown occlusion culling mode" << std::endl;
        }

    }
}


void OcclusionCullingTriMeshRenderer::render_without_oc(Pipeline& pipe, PipelinePassDescription const& desc, 
                                                        scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {
   
        //context means, we are working with the gpu
        RenderContext const& ctx(pipe.get_context());

        SerializedScene& scene = *pipe.current_viewstate().scene;
        auto type_sorted_tri_mesh_node_ptrs_iterator(scene.nodes.find(std::type_index(typeid(node::TriMeshNode)))); //What does this look for?

        //binding G-Buffer and sorting nodes by material->shader??
        if(type_sorted_tri_mesh_node_ptrs_iterator != scene.nodes.end() && !type_sorted_tri_mesh_node_ptrs_iterator->second.empty())
        {
            RenderTarget& render_target = *pipe.current_viewstate().target;
            auto const& camera = pipe.current_viewstate().camera;

            //nach Material sortiert um weniger State Changes zu haben
            std::sort(type_sorted_tri_mesh_node_ptrs_iterator->second.begin(), type_sorted_tri_mesh_node_ptrs_iterator->second.end(), [](node::Node* a, node::Node* b) {
                return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
            });


            bool write_depth = true; //if we set to false, it is all in light greyscale and has a black square projected. Why do we see a bit depth still?
            render_target.bind(ctx, write_depth);
            render_target.set_viewport(ctx);

            scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());


            // currently not needed - for different views?
            //int view_id(camera.config.get_view_id());

            MaterialShader* current_material(nullptr);
            std::shared_ptr<ShaderProgram> current_shader;
            auto current_rasterizer_state = rs_cull_back_; //backface culling
            ctx.render_context->apply();

            // Why by material? Because materials are just pointers and we need less state changes?
            // loop through all objects, sorted by material ----------------------------
            //std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl; 

        //What is the reinterpret cast doing here (converts between types by reinterpreting underlying bit pattern)    
        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc); 
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis(); //check if true or false by keyboard?


        uint object_render_count = 0;
        for(auto const& object : type_sorted_tri_mesh_node_ptrs_iterator->second)
        {
            auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));
            if(pipe.current_viewstate().shadow_mode && tri_mesh_node->get_shadow_mode() == ShadowMode::OFF)
            {
                continue;
            }

            if(!tri_mesh_node->get_render_to_gbuffer()) //sometimes nodes should be invisible and we can get with this function
            {
                continue;
            }

            if(!depth_complexity_vis) { // we render the scene normally if depth complexity visualisation is false
                switch_state_based_on_node_material(ctx, tri_mesh_node, current_shader, current_material, render_target, 
                                                    pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);
            } else {
                switch_state_for_depth_complexity_vis(ctx, current_shader); //rendering with depth complexity on

            }

            if(current_shader && tri_mesh_node->get_geometry()) //How does this work? current_shader is a pointer to a Shader Program? What do we do here?
            {
                upload_uniforms_for_node(ctx, tri_mesh_node, current_shader, pipe, current_rasterizer_state);
                
                tri_mesh_node->get_geometry()->draw(pipe.get_context()); //Here we draw!!! 

                ++object_render_count;
            }
        }

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
        std::cout << "Rendered " << object_render_count << "/" << type_sorted_tri_mesh_node_ptrs_iterator->second.size() << " objects" << std::endl; 
#endif //OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE

        render_target.unbind(ctx); 


        ctx.render_context->reset_state_objects(); //We just reset the state? 
        ctx.render_context->sync();
    }
}


void OcclusionCullingTriMeshRenderer::render_naive_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc, 
                                                                    scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {
   
        // the gua::RenderContext object contains the render_context (~ gl) and the render_device wrapped by schism.
        // the render_device is associated wit the render_context it created.
        //see: include/gua/RenderContext.hpp
        RenderContext const& ctx(pipe.get_context());

        // this is pretty much a standard mechanism of guacamole. You get the Serialized scene and filter the (sorted serialized) nodes by its dynamic type.
        // since we are prototyping the naive stop and wait algorithm without our occlusion culling group node in mind, we look for all trimesh nodes
        SerializedScene& scene = *pipe.current_viewstate().scene;

        // Here we go in the map (of serialize scene) and look for all nodes from the Type TriMesh??? 
        auto type_sorted_node_ptrs_iterator(scene.nodes.find(std::type_index(typeid(node::TriMeshNode))));




        // check if any nodes at all were serialized (nodes that are frustum culled would not appear here) !! Important !!
        if(type_sorted_node_ptrs_iterator != scene.nodes.end() && !type_sorted_node_ptrs_iterator->second.empty())
        {
            // the RenderTarget object target - binding GBuffer
            RenderTarget& render_target = *pipe.current_viewstate().target;
            auto const& camera = pipe.current_viewstate().camera;

            //Sorting based on material to minimize state-changes
            std::sort(type_sorted_node_ptrs_iterator->second.begin(), type_sorted_node_ptrs_iterator->second.end(), [](node::Node* a, node::Node* b) {
                return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
            });


            bool write_depth = true;
            render_target.bind(ctx, write_depth);
            render_target.set_viewport(ctx);

            scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());


            // not needed currently
            //int view_id(camera.config.get_view_id());

            MaterialShader* current_material(nullptr);
            std::shared_ptr<ShaderProgram> current_shader;
            auto current_rasterizer_state = rs_cull_back_;
            ctx.render_context->apply();

            // loop through all objects, sorted by material ----------------------------
            //std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl; 

            
        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();
        bool occlusion_culling_geometry_vis = occlusion_culling_pipeline_pass_description->get_enable_culling_geometry_vis();

        uint64_t object_render_count = 0;
        int rendered_nodes = 0; //will be interesting once we use the OCGroup Node Tree
        


        for(auto const& current_node_ptr : type_sorted_node_ptrs_iterator->second) {
            auto tri_mesh_node_ptr(reinterpret_cast<node::TriMeshNode*>(current_node_ptr));
            if(pipe.current_viewstate().shadow_mode && tri_mesh_node_ptr->get_shadow_mode() == ShadowMode::OFF)
            {
                continue;
            }

            if(!tri_mesh_node_ptr->get_render_to_gbuffer())
            {
                continue;
            }

            auto current_node_path = tri_mesh_node_ptr->get_path();

            bool render_current_node = true;

            // get iterator to occlusion query associated with node (currently by path)
            auto occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_path);

            // if we didn't create an occlusion query for this node (based on path) for this context
            // but we did? So doing it 2 times?
            if(ctx.occlusion_query_objects.end() == occlusion_query_iterator ) {

                // get occlusion query mode

                //default: Number of Sampled Passed -> if 0 it is invisible?
                auto occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_SAMPLES_PASSED;


                // change occlusion query type if we only want to know whether any fragment was visible -> a bool?
                if( OcclusionQueryType::Any_Samples_Passed == desc.get_occlusion_query_type() ) {
                    occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_ANY_SAMPLES_PASSED;
                }

                ctx.occlusion_query_objects.insert(std::make_pair(current_node_path, ctx.render_device->create_occlusion_query(occlusion_query_mode) ) );
            
                occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_path);
            } 


            // now we can be sure that the occlusion query object was created. Start occlusion querying
            {
                current_shader = occlusion_query_box_program_; //Where is this coming from? What does it do?

                current_shader->use(ctx);
                auto vp_mat = view_projection_matrix;
                //retrieve_world_space_bounding_box
                auto world_space_bounding_box = tri_mesh_node_ptr->get_bounding_box();

                //Why do we apply this to the shader? Maybe revisit what a shader does?
                current_shader->apply_uniform(ctx, "view_projection_matrix", math::mat4f(vp_mat));
                current_shader->apply_uniform(ctx, "world_space_bb_min", math::vec3f(world_space_bounding_box.min));
                current_shader->apply_uniform(ctx, "world_space_bb_max", math::vec3f(world_space_bounding_box.max));

                if(!occlusion_culling_geometry_vis) {

                    // IMPORTANT! IN THIS FUNCTION THE COLOR CHANNELS ARE DISABLED AND DEPTH MASK IS DISABLED AS WELL
                    // -> checks for bool from pp-desc,  disables color channels, tests but doesnt write depth
                    set_occlusion_query_states(ctx);
                } else {
                    ctx.render_context->set_depth_stencil_state(default_depth_test_);                        
                }

                ////////////////////// OCCLUSION QUERIES BEGIN -- SUBMIT TO GPU
                ctx.render_context->begin_query(occlusion_query_iterator->second); //second again?
                //pipe knows context
                pipe.draw_box();
                ctx.render_context->end_query(occlusion_query_iterator->second);
  
                // busy waiting - instead of doing something meaningful, we stall the CPU and therefore starve the GPU
                while(!ctx.render_context->query_result_available(occlusion_query_iterator->second)) {
                    ;
                }

                // get query results
                ctx.render_context->collect_query_results(occlusion_query_iterator->second);

                // the result contains the number of sampled that were created (in mode OQMODE_SAMPLES_PASSED) or 0 or 1 (in mode OQMODE_ANY_SAMPLES_PASSED)
                uint64_t query_result = (*occlusion_query_iterator).second->result();


                switch( desc.get_occlusion_query_type() ) {
                    case OcclusionQueryType::Number_Of_Samples_Passed:
                        //So if we define a certain threshold, then check if the number of returned fragments is higher than threshold, only then render (not conservative?)
                        if(query_result > desc.get_occlusion_culling_fragment_threshold()) {
                            ++object_render_count;
                        } else {
                            render_current_node = false;
                        }
                    break;

                    //conservative approach. If any passed we render
                    case OcclusionQueryType::Any_Samples_Passed:
                        if(query_result > 0) {
                            ++object_render_count;
                        } else {
                            render_current_node = false;
                        }
                    break;

                    default: 
                        Logger::LOG_WARNING << "OcclusionCullingTriMeshPass:: unknown occlusion query type encountered." << std::endl;
                    break;
                }
            }



            //be default true, only after occlusion query set to false, if threshold(or any) didnt pass
            if(render_current_node) {
                auto const& glapi = ctx.render_context->opengl_api();

                //enable all color channels again (otherwise we would see nothing) --> resetting states?
                glapi.glColorMask(true, true, true, true);
                ctx.render_context->set_depth_stencil_state(default_depth_test_);
                ctx.render_context->set_blend_state(default_blend_state_);
                ctx.render_context->apply();

                if(!depth_complexity_vis) {
                    switch_state_based_on_node_material(ctx, tri_mesh_node_ptr, current_shader, current_material, render_target, 
                                                        pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);
                } else {
                    switch_state_for_depth_complexity_vis(ctx, current_shader);

                }

                if(current_shader && tri_mesh_node_ptr->get_geometry()) //still dont understand this part
                {
                    upload_uniforms_for_node(ctx, tri_mesh_node_ptr, current_shader, pipe, current_rasterizer_state);
                    tri_mesh_node_ptr->get_geometry()->draw(pipe.get_context());
                }


                ++rendered_nodes;
            }

        }
        
#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
        std::cout << "Rendered " << object_render_count << "/" << type_sorted_node_ptrs_iterator->second.size() << " objects" << std::endl; 
#endif //OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE   

        auto const& glapi = ctx.render_context->opengl_api();
        //reset state before we leave the pass. Direct calls to the glapi object can not be reset by --> this is at the very end of each what? frame?
        glapi.glColorMask(true, true, true, true);
        ctx.render_context->apply();

        render_target.unbind(ctx);

        ctx.render_context->reset_state_objects();
        ctx.render_context->sync();
    }
}

void OcclusionCullingTriMeshRenderer::render_hierarchical_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc, 
                                                                           scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {
    RenderContext const& ctx(pipe.get_context());

    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));


    if(sorted_occlusion_group_nodes != scene.nodes.end() && !sorted_occlusion_group_nodes->second.empty())
    {
        auto& render_target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";
        std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / TrimeshPass";

        pipe.begin_gpu_query(ctx, gpu_query_name);
        pipe.begin_cpu_query(cpu_query_name);
#endif

        bool write_depth = true;
        render_target.bind(ctx, write_depth);
        render_target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());


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
            //    std::cout << "Visited node: " << current_node->get_name() <<  "\t\t Visibility Status: " << current_node->get_visibility(current_cam_node.uuid) << std::endl; 


         
                auto const& glapi = ctx.render_context->opengl_api();
                glapi.glColorMask(true, true, true, true);
                ctx.render_context->set_depth_stencil_state(default_depth_test_);
                ctx.render_context->set_blend_state(default_blend_state_);
                ctx.render_context->apply();

                //make sure that we currently have a trimesh node in our hands
                if( (std::type_index(typeid(node::TriMeshNode)) == std::type_index(typeid(*current_node)) ) ) {

                    //std::cout << "ASSUMING THAT " << current_node->get_name() << " is a trimeshnode" << std::endl;
                    auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(current_node));

                    //if(!depth_complexity_vis) {
                        switch_state_based_on_node_material(ctx, tri_mesh_node, current_shader, current_material, render_target, 
                                                            pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);
                    //} else {
                      //  switch_state_for_depth_complexity_vis(ctx, current_shader);

                    //}

                    if(current_shader && tri_mesh_node->get_geometry())
                    {
                        upload_uniforms_for_node(ctx, tri_mesh_node, current_shader, pipe, current_rasterizer_state);
                        tri_mesh_node->get_geometry()->draw(pipe.get_context());
                    }


                    //++rendered_nodes;
            
                }

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
        




        render_target.unbind(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
        pipe.end_gpu_query(ctx, gpu_query_name);
        pipe.end_cpu_query(cpu_query_name);
#endif


        auto const& glapi = ctx.render_context->opengl_api();
        glapi.glColorMask(true, true, true, true);
        ctx.render_context->set_depth_stencil_state(default_depth_test_);
        ctx.render_context->set_blend_state(default_blend_state_);
        ctx.render_context->apply();

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
void OcclusionCullingTriMeshRenderer::set_occlusion_query_states(RenderContext const& ctx) {
    auto const& glapi = ctx.render_context->opengl_api();

    // we disable all color channels to save rasterization time
    glapi.glColorMask(false, false, false, false);

    // set depth state that tests, but does not write depth (otherwise we would have bounding box contours in the depth buffer -> not conservative anymore)
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_test_without_writing_state_);
    ctx.render_context->apply();
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::switch_state_based_on_node_material(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader, 
                                                                          MaterialShader* current_material, RenderTarget const& render_target, bool shadow_mode, std::size_t cam_uuid) {
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
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::switch_state_for_depth_complexity_vis(RenderContext const& ctx, std::shared_ptr<ShaderProgram>& current_shader) {
    if(current_shader != depth_complexity_vis_program_) {
        current_shader = depth_complexity_vis_program_;
        current_shader->use(ctx);
    }


    if(    ctx.render_context->current_blend_state() != color_accumulation_state_
        || ctx.render_context->current_depth_stencil_state() != default_depth_test_)
    {
        ctx.render_context->set_blend_state(color_accumulation_state_);
        ctx.render_context->set_depth_stencil_state(default_depth_test_);
        ctx.render_context->apply_state_objects();
    }
}



////////////////////////////////////////////////////////////////////////////////

} // namespace gua
