#include <gua/renderer/OcclusionCullingTriMeshRenderer.hpp>
#include <gua/renderer/OcclusionCullingTriMeshPass.hpp>

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>
#include <gua/node/OcclusionCullingGroupNode.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>

#include <boost/assign/list_of.hpp>
#include <scm/gl_core/render_device/opengl/gl_core.h>


namespace gua
{
#define USE_PRIORITY_QUEUE


void OcclusionCullingTriMeshRenderer::render_hierarchical_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {

    //REMEMBER > can get distance to camera already do not implement ourselves. get from ctx or viewstate
    RenderContext const& ctx(pipe.get_context());

    auto& scene = *pipe.current_viewstate().scene;

    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));


    auto const& culling_frustum = pipe.current_viewstate().frustum;

    //if oc-group note existed in map of serialized scene and is not empty
    if(sorted_occlusion_group_nodes != scene.nodes.end() && !sorted_occlusion_group_nodes->second.empty())
    {
        auto& render_target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;


        bool write_depth = true;
        render_target.bind(ctx, write_depth);
        render_target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());


        int view_id(camera.config.get_view_id());

        MaterialShader* current_material(nullptr);
        std::shared_ptr<ShaderProgram> current_shader;
        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();

        //std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl;

        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();
        bool occlusion_culling_geometry_vis = occlusion_culling_pipeline_pass_description->get_enable_culling_geometry_vis();

        // get a (serialized) cam node (see: guacamole/src/gua/node/CameraNode.cpp and guacamole/src/gua/node/CameraNode.hpp)
        auto const& current_cam_node = pipe.current_viewstate().camera;
        //std::cout << "Current Cam UUID: " << current_cam_node.uuid << std::endl;

        // reset visibility status if the node was never visited
        // only one element is in the vector. is it necessary to do a for loop here? Or for the case we have more than one oc-group_node
        for( auto& occlusion_group_node : sorted_occlusion_group_nodes->second ) {

            // if we never checked set the visibility status for this node, it will be 0.
            // in this case, we recursively set the entire hierarchy to visible

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
            std::cout << "Camera UUID: " << current_cam_node.uuid << std::endl;
#endif
            // last checked frame id
            int32_t frame_id = get_last_visibility_check_frame_id(occlusion_group_node->unique_node_id(), current_cam_node.uuid);

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
            std::cout << "FRAME ID: " << frame_id << std::endl;
#endif
            if( 0 != frame_id) {
                continue;
            }

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
            std::cout << "RESET VISIBILITY " << std::endl;
#endif
            //std::cout << "Setting hierarchy to visible" << std::endl;
            // use a queue for breadth first search (stack would do breadth first search)
            std::queue<gua::node::Node*> traversal_queue;

            // add root node of our occlusion hierarchy to the traversal queue
            traversal_queue.push(occlusion_group_node);



            //this parts traverses the tree and sets all nodes to visible
            while(!traversal_queue.empty()) {
                // get next node
                gua::node::Node* current_node = traversal_queue.front();
                // work on it (CHC-Style)
                set_visibility(current_node->unique_node_id(), current_cam_node.uuid, true);

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


            // use a queue for breadth first search (stack would do breadth first search)

#ifdef USE_PRIORITY_QUEUE
            std::priority_queue<std::pair<gua::node::Node*, double>,
                std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator > traversal_priority_queue;
            //    { return lhs.second < rhs.second;> traversal_queue;
#else
            std::queue<std::pair<gua::node::Node*, double> > traversal_priority_queue;
#endif

            int rendered_nodes = 0;
            int tested_nodes = 0;

            int total_num_nodes = 0;
            // add root node of our occlusion hierarchy to the traversal queue

            auto node_distance_pair_to_insert = std::make_pair(occlusion_group_node,
                                                scm::math::length_sqr(world_space_cam_pos - (occlusion_group_node->get_bounding_box().max + occlusion_group_node->get_bounding_box().min)/2.0f ) );


            traversal_priority_queue.push(node_distance_pair_to_insert);


            bool is_first_traversed_node = true;

            while(!traversal_priority_queue.empty()) {
                // get next node

#ifdef USE_PRIORITY_QUEUE
                std::pair<gua::node::Node*, double> current_node_dist_pair = traversal_priority_queue.top();
#else
                std::pair<gua::node::Node*, double> current_node_dist_pair = traversal_priority_queue.front();
#endif
                //remove this node from the queue
                traversal_priority_queue.pop();

                gua::node::Node* current_node = current_node_dist_pair.first;



                bool is_in_frustum = culling_frustum.intersects(current_node->get_bounding_box());

                if (is_in_frustum) {


                    auto current_node_id = current_node_dist_pair.first->unique_node_id();

                    // get iterator to occlusion query associated with node (currently by path)
                    auto occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);

                    // if we didn't create an occlusion query for this node (based on path) for this context
                    // should only happen if a node has not been in frustum, because in map there is no oc-query object yet
                    if(ctx.occlusion_query_objects.end() == occlusion_query_iterator ) {

                        // get occlusion query mode
                        //std::cout << "Creating occlusion query for node with path: " << current_node_path << std::endl;
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


                    current_shader = occlusion_query_box_program_; //define a new shader of type occlusion query box program

                    current_shader->use(ctx);
                    auto vp_mat = view_projection_matrix;
                    //retrieve_world_space_bounding_box
                    auto world_space_bounding_box = current_node->get_bounding_box();

                    //We want to render the red boxes
                    current_shader->apply_uniform(ctx, "view_projection_matrix", math::mat4f(vp_mat));
                    current_shader->apply_uniform(ctx, "world_space_bb_min", math::vec3f(world_space_bounding_box.min));
                    current_shader->apply_uniform(ctx, "world_space_bb_max", math::vec3f(world_space_bounding_box.max));


                    if(occlusion_culling_geometry_vis) {
                        switch_state_for_depth_complexity_vis(ctx, current_shader);
                    } else {
                        //render_context->set_depth_stencil_state(default_depth_test_);
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

                    ctx.render_context->collect_query_results(occlusion_query_iterator->second);

                    // the result contains the number of sampled that were created (in mode OQMODE_SAMPLES_PASSED) or 0 or 1 (in mode OQMODE_ANY_SAMPLES_PASSED)
                    uint64_t query_result = (*occlusion_query_iterator).second->result();

                    

                    if(is_first_traversed_node) {
                        is_first_traversed_node = false;

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
                        std::cout << "Camera UUID 2: " << current_cam_node.uuid << std::endl;

                        // current frame id
                        std::cout << "current frame_id: " << frame_id << std::endl;
#endif
                    }


                    if (query_result > 0) {
                        if (current_node->get_children().size()>0) {
                            //interior node

                            for(std::shared_ptr<gua::node::Node> const& shared_child_node_ptr : current_node->get_children()) {
                                auto child_node_distance_pair
                                    = std::make_pair(shared_child_node_ptr.get(), scm::math::length_sqr(world_space_cam_pos - (shared_child_node_ptr->get_bounding_box().max + shared_child_node_ptr->get_bounding_box().min)/2.0f ) );
                                traversal_priority_queue.push(child_node_distance_pair);
                            }


                        } else {
                            ++rendered_nodes;
                            //leaf node
                            render_visible_leaf(current_node,
                                                ctx, pipe,
                                                render_target,
                                                current_material,
                                                current_shader,
                                                current_rasterizer_state,
                                                depth_complexity_vis);
                        }
                    }
                }

            }
            //std::cout << "# tested nodes: " << tested_nodes << "/" << total_num_nodes  << std::endl;
            //std::cout<< "all rendered nodes " << rendered_nodes <<std::endl;
        }

        // also the current
        unbind_and_reset(ctx, render_target);

    }
}

}