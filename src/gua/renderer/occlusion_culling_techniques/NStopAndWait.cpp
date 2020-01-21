#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/node/OcclusionCullingGroupNode.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>

#include <boost/assign/list_of.hpp>
#include <scm/gl_core/render_device/opengl/gl_core.h>


namespace gua {

void OcclusionCullingTriMeshRenderer::render_naive_stop_and_wait_oc(
    Pipeline& pipe,
    PipelinePassDescription const& desc,
    scm::math::mat4d const& view_projection_matrix,
    gua::math::vec3f const& world_space_cam_pos) {

    // the gua::RenderContext object contains the render_context (~ gl) and the render_device wrapped by schism.
    // the render_device is associated wit the render_context it created.
    //see: include/gua/RenderContext.hpp
    RenderContext const& ctx(pipe.get_context());

    // this is pretty much a standard mechanism of guacamole. You get the Serialized scene and filter the (sorted serialized) nodes by its dynamic type.
    // since we are prototyping the naive stop and wait algorithm without our occlusion culling group node in mind, we look for all trimesh nodes
    SerializedScene& scene = *pipe.current_viewstate().scene;

    // Here we go in the map (of serialize scene) and look for all nodes from the Type TriMesh
    auto type_sorted_node_ptrs_iterator(scene.nodes.find(std::type_index(typeid(node::TriMeshNode))));



    // check if any nodes at all were serialized (nodes that are frustum culled would not appear here) !! Important !!
    if(type_sorted_node_ptrs_iterator != scene.nodes.end() && !type_sorted_node_ptrs_iterator->second.empty())
    {
        // the RenderTarget object target - binding GBuffer
        RenderTarget& render_target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

        //Sorting based on material to minimize state-changes

        bool write_depth = true;
        render_target.bind(ctx, write_depth);
        render_target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());


        MaterialShader* current_material(nullptr);
        std::shared_ptr<ShaderProgram> current_shader;
        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();

        // loop through all objects, sorted by material ----------------------------

        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();
        bool occlusion_culling_geometry_vis = occlusion_culling_pipeline_pass_description->get_enable_culling_geometry_vis();

        uint64_t object_render_count = 0;
        int rendered_nodes = 0; //will be interesting once we use the OCGroup Node Tree

        std::vector<std::pair<gua::node::Node*, double> > node_distance_pair_vector;

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


            auto node_distance_pair_to_insert = std::make_pair(current_node_ptr,
                                                scm::math::length_sqr(world_space_cam_pos - (current_node_ptr->get_bounding_box().max + current_node_ptr->get_bounding_box().min)/2.0f ) );

            node_distance_pair_vector.push_back(node_distance_pair_to_insert);


            std::sort(node_distance_pair_vector.begin(), node_distance_pair_vector.end(),
            [](std::pair<gua::node::Node*, double> const& lhs, std::pair<gua::node::Node*, double> const& rhs) {
                return lhs.second < rhs.second;
            }   );
        }


        for(auto const& current_node_distance_pair : node_distance_pair_vector) {


            gua::node::TriMeshNode* current_node = reinterpret_cast<node::TriMeshNode*>(current_node_distance_pair.first);

            auto current_node_id = current_node->unique_node_id();

            auto occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);

            if(ctx.occlusion_query_objects.end() == occlusion_query_iterator ) {

                // get occlusion query mode
#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
                std::cout << "Creating occlusion query for node with unique_node_id: " << current_node_id << std::endl;
#endif
                //default: Number of Sampled Passed -> if 0 it is invisible?
                auto occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_SAMPLES_PASSED;


                // change occlusion query type if we only want to know whether any fragment was visible -> a bool?
                if( OcclusionQueryType::Any_Samples_Passed == desc.get_occlusion_query_type() ) {
                    occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_ANY_SAMPLES_PASSED;
                }

                //Fuegen neues object (path, query object) zur map im ctx hinzu    //Damit wir Iterator an der Hand haben
                ctx.occlusion_query_objects.insert(std::make_pair(current_node_id, ctx.render_device->create_occlusion_query(occlusion_query_mode) ) );

                occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);
            }

            bool render_current_node = true;

            current_shader = occlusion_query_box_program_; //Red boxes

            current_shader->use(ctx);
            auto vp_mat = view_projection_matrix;
            //retrieve_world_space_bounding_box

            auto world_space_bounding_box = current_node->get_bounding_box();
            //We want to render the red boxes
            current_shader->apply_uniform(ctx, "view_projection_matrix", math::mat4f(vp_mat));
            current_shader->apply_uniform(ctx, "world_space_bb_min", math::vec3f(world_space_bounding_box.min));
            current_shader->apply_uniform(ctx, "world_space_bb_max", math::vec3f(world_space_bounding_box.max));

            if(occlusion_culling_geometry_vis) {
                ctx.render_context->set_depth_stencil_state(depth_stencil_state_no_test_no_writing_state_);
            } else {
                // IMPORTANT! IN THIS FUNCTION THE COLOR CHANNELS ARE DISABLED AND DEPTH MASK IS DISABLED AS WELL
                // -> checks for bool from pp-desc,  disables color channels, tests but doesnt write depth
                set_occlusion_query_states(ctx);
            }

            ////////////////////// OCCLUSION QUERIES BEGIN -- SUBMIT TO GPU
            ctx.render_context->begin_query(occlusion_query_iterator->second); //second is the query object from the map
            //pipe knows context
            pipe.draw_box(); // when we draw, the query is admitted for all drawn objects. in current_shader we create bb

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


            if(render_current_node) {

                auto const& glapi = ctx.render_context->opengl_api();

                //enable all color channels again (otherwise we would see nothing) --> resetting states
                glapi.glColorMask(true, true, true, true);
                ctx.render_context->set_depth_stencil_state(default_depth_test_);
                ctx.render_context->set_blend_state(default_blend_state_);
                ctx.render_context->apply();

                if(depth_complexity_vis) {
                    switch_state_for_depth_complexity_vis(ctx, current_shader);
                } else {

                    switch_state_based_on_node_material(ctx, current_node, current_shader, current_material, render_target,
                                                        pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);

                }

                if(current_shader && current_node->get_geometry())
                {
                    upload_uniforms_for_node(ctx, current_node, current_shader, pipe, current_rasterizer_state);
                    current_node->get_geometry()->draw(pipe.get_context());
                }

                ++rendered_nodes;
            }

        }

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
        std::cout << "Rendered " << object_render_count << "/" << type_sorted_node_ptrs_iterator->second.size() << " objects" << std::endl;
#endif //OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE   
        unbind_and_reset(ctx, render_target);

    }
}
}