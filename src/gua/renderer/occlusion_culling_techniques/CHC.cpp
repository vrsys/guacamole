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


bool query_context_state = false;

namespace gua
{
#define USE_PRIORITY_QUEUE

void OcclusionCullingTriMeshRenderer::render_CHC(Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
    std::cout << "Start of new Frame " << std::endl;
#endif
    RenderContext const& ctx(pipe.get_context());
    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));


    //if oc-group note existed in map of serialized scene and is not empty
    if (sorted_occlusion_group_nodes != scene.nodes.end() && !sorted_occlusion_group_nodes->second.empty())
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

        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();
        bool occlusion_culling_geometry_vis = occlusion_culling_pipeline_pass_description->get_enable_culling_geometry_vis();

        // get a (serialized) cam node (see: guacamole/src/gua/node/CameraNode.cpp and guacamole/src/gua/node/CameraNode.hpp)
        auto const& current_cam_node = pipe.current_viewstate().camera;
        //std::cout << "Current Cam UUID: " << current_cam_node.uuid << std::endl;

        //query queue for nodes that were previously invisible and need to be queried

        std::queue<std::pair<gua::node::Node*, scm::gl::occlusion_query_ptr> > i_query_queue;

        //query queue for nodes that were previously visible and need to be queried but can wait until the end of the frame
        std::queue<std::pair<gua::node::Node*, scm::gl::occlusion_query_ptr> > v_query_queue;

        // reset visibility status if the node was never visited
        // only one element is in the vector. is it necessary to do a for loop here? Or for the case we have more than one oc-group_node
        for ( auto& occlusion_group_node : sorted_occlusion_group_nodes->second )
        {

            int32_t last_visibility_check_frame_id = get_last_visibility_check_frame_id(occlusion_group_node->unique_node_id(), current_cam_node.uuid);
            // if we never checked set the visibility status for this node, it will be 0.
            // in this case, we recursively set the entire hierarchy to visible

            if ( 0 != last_visibility_check_frame_id) {
                continue;
            }

            std::queue<gua::node::Node*> traversal_queue;

            // add root node of our occlusion hierarchy to the traversal queue
            traversal_queue.push(occlusion_group_node);
            // this parts traverses the tree and sets all nodes to visible
            // creates occlusion query objects
            // add all to query queue

            //INITIALIZATION
            while (!traversal_queue.empty()) {
                // get next node
                gua::node::Node* current_node = traversal_queue.front();
                set_visibility(current_node->unique_node_id(), current_cam_node.uuid, true);

                traversal_queue.pop();

                //push all children (currently in arbitrary order)
                for (std::shared_ptr<gua::node::Node> const& shared_child_node_ptr : current_node->get_children()) {

                    // the vector returned by "get_children" unfortunately contains shared_ptrs instead of raw ptrs.
                    // the serializer however creates raw prts. Calling ".get()" on a shared ptr provides us with the
                    // raw ptr that is referenced by the manager object
                    gua::node::Node* raw_child_node_ptr = shared_child_node_ptr.get();
                    traversal_queue.push(raw_child_node_ptr);

                }
            }
        }


        // ACTUAL CHC IMPLEMENTATION (w/o/ initializaton)

#ifdef USE_PRIORITY_QUEUE
        std::priority_queue<std::pair<gua::node::Node*, double>,
            std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator > traversal_priority_queue;
#else
        std::queue<std::pair<gua::node::Node*, double> > traversal_priority_queue;
#endif

        int64_t const current_frame_id = ctx.framecount;

        auto const& culling_frustum = pipe.current_viewstate().frustum;

        //each of the occlusion group nodes models an entire object hierarchy
        for (auto const& occlusion_group_node : sorted_occlusion_group_nodes->second)
        {

            // use a queue for breadth first search (stack would do breadth first search)
            int rendered_nodes = 0;
            int total_num_trimesh_nodes = 0;

            // add root node of our occlusion hierarchy to the traversal queue
            auto node_distance_pair_to_insert = std::make_pair(occlusion_group_node,
                                                scm::math::length_sqr(world_space_cam_pos - (occlusion_group_node->get_bounding_box().max + occlusion_group_node->get_bounding_box().min) / 2.0f ) );

            traversal_priority_queue.push(node_distance_pair_to_insert);

            while (!traversal_priority_queue.empty() || !i_query_queue.empty() )
            {

                //------------------------------------------------------------------PART 1
                bool result_available = false;
                if (!i_query_queue.empty()) {
                    result_available = ctx.render_context->query_result_available(i_query_queue.front().second);
                }

                while (!i_query_queue.empty() && (result_available || traversal_priority_queue.empty()))
                {

                    auto current_query_node = i_query_queue.front().first;
                    auto front_query_obj_queue = i_query_queue.front().second;

                    i_query_queue.pop();


                    // waiting for the result of the previously invisible node
                    while (!result_available)
                    {
                        result_available = ctx.render_context->query_result_available(front_query_obj_queue);
                    }



                    ctx.render_context->collect_query_results(front_query_obj_queue);

                    //int64_t last_checked_frame_id_for_node = get_last_visibility_check_frame_id(current_query_node->unique_node_id(), current_cam_node.uuid);

                    // the result contains the number of sampled that were created (in mode OQMODE_SAMPLES_PASSED) or 0 or 1 (in mode OQMODE_ANY_SAMPLES_PASSED)
                    uint64_t query_result = front_query_obj_queue->result();


                    if (current_query_node->get_children().empty()) {
                        ++total_num_trimesh_nodes;
                    }


                    bool render_current_node = false;


                    switch ( desc.get_occlusion_query_type() ) {
                    case OcclusionQueryType::Number_Of_Samples_Passed:
                        //So if we define a certain threshold, then check if the number of returned fragments is higher than threshold, only then render (not conservative?)
                        if (query_result > desc.get_occlusion_culling_fragment_threshold()) {
                            render_current_node = true;
                        } else {
                            render_current_node = false;
                        }
                        break;

                    //conservative approach. If any passed we render
                    case OcclusionQueryType::Any_Samples_Passed:
                        if (query_result > 0) {
                            render_current_node = true;
                        } else {
                            render_current_node = false;
                        }
                        break;

                    default:
                        Logger::LOG_WARNING << "OcclusionCullingTriMeshPass:: unknown occlusion query type encountered." << std::endl;
                        break;
                    }

                    if (render_current_node) //query_result > 0 )
                    {
                        // pull up visibility
                        pull_up_visibility(current_query_node, current_cam_node.uuid);

                        // For interior Nodes
                        if (!current_query_node->get_children().empty())
                        {
                            // Traverse the interior Nodes
                            for (auto& child : current_query_node->get_children()) {

                                auto child_node_distance_pair_to_insert = std::make_pair(child.get(), scm::math::length_sqr(world_space_cam_pos - (child->get_bounding_box().max + child->get_bounding_box().min) / 2.0f ) );
                                traversal_priority_queue.push(child_node_distance_pair_to_insert);
                            }
                        } else {

                            // render curent_node
                            ++rendered_nodes;
                            render_visible_leaf(current_query_node,
                                                ctx, pipe,
                                                render_target,
                                                current_material,
                                                current_shader,
                                                current_rasterizer_state,
                                                depth_complexity_vis);

                        }
                    }

                }

//---------------------------------------------------------------PART 2

                if (!traversal_priority_queue.empty())
                {
#ifdef USE_PRIORITY_QUEUE
                    auto current_node = traversal_priority_queue.top().first;
#else
                    auto current_node = traversal_priority_queue.front().first;
#endif // USE_PRIORITY_QUEUE
                    traversal_priority_queue.pop();

                    //std::cout<<"begin tq: "<< ctx.framecount <<" current node " << current_node->get_name() <<
                    //" is  " << get_visibility(current_node->unique_node_id(), current_cam_node.uuid)<<std::endl;


                    auto distance_cam_to_node = scm::math::length_sqr(world_space_cam_pos - (current_node->get_bounding_box().max
                                                + current_node->get_bounding_box().min) / 2.0f ) ;
#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
                    std::cout << "Traversing node: " << current_node->get_name() << "  with dist to cam: " << distance_cam_to_node << std::endl;
#endif

                    bool is_in_frustum = culling_frustum.intersects(current_node->get_bounding_box());

                    if (is_in_frustum)
                    {

                        bool visibility_current_node = get_visibility(current_node->unique_node_id(), current_cam_node.uuid);

                        int32_t node_last_checked_frame_id = get_last_visibility_check_frame_id(current_node->unique_node_id(), current_cam_node.uuid);

                        bool was_visible = (visibility_current_node &&
                                            (node_last_checked_frame_id == 0 ||
                                             (node_last_checked_frame_id == current_frame_id - 1)) );

                        bool is_opened = (!current_node->get_children().empty() && was_visible);

                        set_visibility(current_node->unique_node_id(), current_cam_node.uuid, false);
                        set_last_visibility_check_frame_id(current_node->unique_node_id(), current_cam_node.uuid, current_frame_id);

                        if (!is_opened)
                        {
                            auto current_node_id = current_node->unique_node_id();

                            /////// Check whether occlusion query objects for this node were already created
                            auto occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);

                            if (ctx.occlusion_query_objects.end() == occlusion_query_iterator ) {
                                // get occlusion query mode
                                //default: Number of Sampled Passed -> if 0 it is invisible?
                                auto occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_SAMPLES_PASSED;


                                // change occlusion query type if we only want to know whether any fragment was visible
                                if ( OcclusionQueryType::Any_Samples_Passed == desc.get_occlusion_query_type() ) {
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

                            if (occlusion_culling_geometry_vis) {
                                switch_state_for_depth_complexity_vis(ctx, current_shader);
                            } else {
                                if (!query_context_state){
                                    set_occlusion_query_states(ctx);
                                }
                                
                            }

                            auto current_occlusion_query_object = occlusion_query_iterator->second;
                            // begin query?
                            ctx.render_context->begin_query(current_occlusion_query_object); //second is the query object from the map
                            //pipe knows context
                            pipe.draw_box(); // when we draw, the query is admitted for all drawn objects. in current_shader we create bb
                            ctx.render_context->end_query(current_occlusion_query_object);

                            if (!was_visible) {
                                i_query_queue.push(std::make_pair(current_node, current_occlusion_query_object) );
                            } else {
                                v_query_queue.push(std::make_pair(current_node, current_occlusion_query_object) );
                            }
                        }


                        if (was_visible)
                        {
                            if (!(current_node->get_children().empty())) {

                                // Traverse the nodes
                                for (auto & child : current_node->get_children())
                                {

                                    auto child_node_distance_pair_to_insert = std::make_pair(child.get(), scm::math::length_sqr(world_space_cam_pos - (child->get_bounding_box().max + child->get_bounding_box().min) / 2.0f ) );
                                    traversal_priority_queue.push(child_node_distance_pair_to_insert);
                                }

                            } else {

                                ++rendered_nodes;
                                //render current node
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

            }

            while (!v_query_queue.empty()) {

                auto current_query_node = v_query_queue.front().first;
                auto front_query_obj_queue = v_query_queue.front().second;

                v_query_queue.pop();



/////////////////////////////////


                // busy waiting - instead of doing something meaningful, we stall the CPU and therefore starve the GPU

                bool result_available = false;
                while (!result_available)
                {
                    result_available = ctx.render_context->query_result_available(front_query_obj_queue);
                }


                ctx.render_context->collect_query_results(front_query_obj_queue);

                //int64_t last_checked_frame_id_for_node = get_last_visibility_check_frame_id(current_query_node->unique_node_id(), current_cam_node.uuid);

                // the result contains the number of sampled that were created (in mode OQMODE_SAMPLES_PASSED) or 0 or 1 (in mode OQMODE_ANY_SAMPLES_PASSED)
                uint64_t query_result = front_query_obj_queue->result();


                bool render_current_node = false;


                switch ( desc.get_occlusion_query_type() ) {
                case OcclusionQueryType::Number_Of_Samples_Passed:
                    //So if we define a certain threshold, then check if the number of returned fragments is higher than threshold, only then render (not conservative?)
                    if (query_result > desc.get_occlusion_culling_fragment_threshold()) {
                        render_current_node = true;
                    } else {
                        render_current_node = false;
                    }
                    break;

                //conservative approach. If any passed we render
                case OcclusionQueryType::Any_Samples_Passed:
                    if (query_result > 0) {
                        render_current_node = true;
                    } else {
                        render_current_node = false;
                    }
                    break;

                default:
                    Logger::LOG_WARNING << "OcclusionCullingTriMeshPass:: unknown occlusion query type encountered." << std::endl;
                    break;
                }

                if (render_current_node) //query_result > 0  )
                {

                    set_visibility(current_query_node->unique_node_id(), current_cam_node.uuid, true);


                }
/////////////////////////////////
            }

        }

        unbind_and_reset(ctx, render_target);

    }

}

} // namespace gua