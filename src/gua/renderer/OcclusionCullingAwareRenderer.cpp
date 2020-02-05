#include <gua/renderer/OcclusionCullingAwareRenderer.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <boost/assign/list_of.hpp>
#include <scm/gl_core/render_device/opengl/gl_core.h>
#include <gua/node/OcclusionCullingGroupNode.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/config.hpp>
#include <gua/renderer/Pipeline.hpp>



std::array<float, 16> keep_probability;
uint64_t batch_size_multi_query = 10;

#define USE_PRIORITY_QUEUE
#define DYNAMIC_BATCH_SIZE
#define RENDER_CHILD_CLASS
#define query_state_test

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

}


namespace gua
{


OcclusionCullingAwareRenderer::OcclusionCullingAwareRenderer(RenderContext const& ctx, SubstitutionMap const& smap) :
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
    occlusion_query_array_box_program_stages_(), occlusion_query_array_box_program_(nullptr), //only one shader that is independent of the actual node material
    global_substitution_map_(smap) {
    	
    	for(int32_t used_frames_index = 0; used_frames_index < keep_probability.size(); ++used_frames_index) {
    		keep_probability[used_frames_index] = 0.99 - 0.7 * std::exp(-used_frames_index);
		}

	#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
	    ResourceFactory factory;
	    std::string v_occlusion_query_box = factory.read_shader_file("resources/shaders/occlusion_query_box.vert");
	    std::string f_occlusion_query_box = factory.read_shader_file("resources/shaders/occlusion_query_box.frag");

	    // a new array box shader for CHC++
	    std::string v_occlusion_query_array_box = factory.read_shader_file("resources/shaders/occlusion_query_array_box.vert");
	    std::string f_occlusion_query_array_box = factory.read_shader_file("resources/shaders/occlusion_query_array_box.frag");

	    std::string v_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
	    std::string f_default_rendering_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");

	    std::string v_depth_complexity_vis = factory.read_shader_file("resources/shaders/tri_mesh_shader_no_programmable_material.vert");
	    std::string f_depth_complexity_vis = factory.read_shader_file("resources/shaders/depth_complexity_to_color.frag");
	#else
	    std::string v_occlusion_query_box = Resources::lookup_shader("shaders/occlusion_query_box.vert");
	    std::string f_occlusion_query_box = Resources::lookup_shader("shaders/occlusion_query_box.frag");

	    // a new array box shader for CHC++
	    std::string v_occlusion_query_array_box = Resources::lookup_shader("shaders/occlusion_query_array_box.vert");
	    std::string v_occlusion_query_array_box = Resources::lookup_shader("shaders/occlusion_query_array_box.frag");

	    std::string v_default_rendering_vis = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
	    std::string f_default_rendering_vis = Resources::lookup_shader("shaders/tri_mesh_shader.frag");

	    std::string v_depth_complexity_vis = Resources::lookup_shader("shaders/tri_mesh_shader_no_programmable_material.vert");
	    std::string f_depth_complexity_vis = Resources::lookup_shader("shaders/depth_complexity_to_color.frag");
	#endif

	    default_rendering_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_default_rendering_vis);
	    default_rendering_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_default_rendering_vis);

	    occlusion_query_box_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_occlusion_query_box);
	    occlusion_query_box_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_occlusion_query_box);

	    // new shader program stages for CHC++
	    occlusion_query_array_box_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_occlusion_query_array_box);
	    occlusion_query_array_box_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_occlusion_query_array_box);

	    depth_complexity_vis_program_stages_.emplace_back(scm::gl::STAGE_VERTEX_SHADER, v_depth_complexity_vis);
	    depth_complexity_vis_program_stages_.emplace_back(scm::gl::STAGE_FRAGMENT_SHADER, f_depth_complexity_vis);


	    std::cout << "Recreated trimesh renderer" << std::endl;
    }



void OcclusionCullingAwareRenderer::render_switch_occlusion_culling(Pipeline& pipe, PipelinePassDescription const& desc) {
        switch (desc.get_occlusion_culling_strategy()) {
        case OcclusionCullingStrategy::No_Culling: {
            render(pipe, desc);
            break;
        }
        case OcclusionCullingStrategy::Coherent_Hierarchical_Culling_PlusPlus: {
            render_with_occlusion_culling(pipe, desc);
            break;
        } default: {
            std::cout << "Unknown occlusion culling mode" << std::endl;
        }
    }

}

void OcclusionCullingAwareRenderer::render_with_occlusion_culling(Pipeline& pipe, PipelinePassDescription const& desc){

    
    RenderContext const& ctx(pipe.get_context());


    if( nullptr == empty_vbo_) {
        empty_vbo_ = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, 0, 0);
        ctx.render_context->apply_vertex_input();
    }

    // size_t size_of_vertex = 2 * sizeof(uint32_t);
    if(nullptr == empty_vao_layout_) {
        empty_vao_layout_ = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_UINT, 0), boost::assign::list_of(empty_vbo_));
    }


    if(nullptr == occlusion_query_box_program_) {
        occlusion_query_box_program_ = std::make_shared<ShaderProgram>();
        occlusion_query_box_program_->set_shaders(occlusion_query_box_program_stages_, std::list<std::string>(), false, global_substitution_map_);
    }

    if(nullptr == depth_complexity_vis_program_) {
        depth_complexity_vis_program_ = std::make_shared<ShaderProgram>();
        depth_complexity_vis_program_->set_shaders(depth_complexity_vis_program_stages_, std::list<std::string>(), false, global_substitution_map_);
    }

    if (nullptr == occlusion_query_array_box_program_)
    {
        occlusion_query_array_box_program_ = std::make_shared<ShaderProgram>();
        occlusion_query_array_box_program_ ->set_shaders(occlusion_query_array_box_program_stages_, std::list<std::string>(), false, global_substitution_map_);
    }


    
    auto const& frustum = pipe.current_viewstate().frustum;
    scm::math::mat4d const view_matrix = frustum.get_view();
    scm::math::mat4d const projection_matrix = frustum.get_projection();
    scm::math::mat4d view_projection_matrix = projection_matrix * view_matrix;

    

    scm::math::mat4d const camera_matrix = scm::math::inverse(view_matrix);

    gua::math::vec4f camera_space_cam_pos_homogeneous(0.0f, 0.0f, 0.0f, 1.0f);
    gua::math::vec4f world_space_cam_pos_homogeneous = gua::math::mat4f(camera_matrix) * camera_space_cam_pos_homogeneous;

    gua::math::vec3f world_space_cam_pos(world_space_cam_pos_homogeneous[0], world_space_cam_pos_homogeneous[1], world_space_cam_pos_homogeneous[2]);

    auto const pipeline_pass_description = reinterpret_cast<PipelinePassDescription const*>(&desc);
    


    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));


    
//if oc-group note existed in map of serialized scene and is not empty
    if(sorted_occlusion_group_nodes != scene.nodes.end() && !sorted_occlusion_group_nodes->second.empty())
    {

        auto& render_target = *pipe.current_viewstate().target;

        bool write_depth = true;
        render_target.bind(ctx, write_depth);
        render_target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());

        //int view_id(camera.config.get_view_id());
        MaterialShader* current_material(nullptr);
        std::shared_ptr<ShaderProgram> current_shader;
        auto current_rasterizer_state = rs_cull_back_;
        ctx.render_context->apply();


        depth_complexity_visualization_ = pipeline_pass_description->get_enable_depth_complexity_vis();
        occlusion_culling_geometry_visualization_ = pipeline_pass_description->get_enable_culling_geometry_vis();
        
        // get a (serialized) cam node (see: guacamole/src/gua/node/CameraNode.cpp and guacamole/src/gua/node/CameraNode.hpp)
        auto const& current_cam_node = pipe.current_viewstate().camera;

        std::queue<MultiQuery> query_queue;
        std::queue<gua::node::Node*> i_query_queue;
        std::queue<gua::node::Node*> v_query_queue;
        std::queue<gua::node::Node*> visibility_setting_queue;

        

        // reset visibility status if the node was never visited
        for( auto& occlusion_group_node : sorted_occlusion_group_nodes->second )
        {

            
            int32_t last_visibility_check_frame_id = get_last_visibility_check_frame_id(occlusion_group_node->unique_node_id(), current_cam_node.uuid);
            // if we never checked set the visibility status for this node, it will be 0.
            // in this case, we recursively set the entire hierarchy to visible

            if( 0 != last_visibility_check_frame_id) {
                continue;
            }

            std::queue<gua::node::Node*> traversal_queue;

            // add root node of our occlusion hierarchy to the traversal queue
            traversal_queue.push(occlusion_group_node);
            // this parts traverses the tree and sets all nodes to visible



            //INITIALIZATION
            while(!traversal_queue.empty()) {
                
                // get next node
                gua::node::Node* current_node = traversal_queue.front();
                set_visibility(current_node->unique_node_id(), current_cam_node.uuid, true);

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

        
        // ACTUAL CHC++ IMPLEMENTATION (w/o/ initializaton)********************************************************************************************************************************************************

#ifdef USE_PRIORITY_QUEUE
        std::priority_queue<std::pair<gua::node::Node*, double>,
            std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator > traversal_priority_queue;
#else
        std::queue<std::pair<gua::node::Node*, double> > traversal_priority_queue;
#endif

        int64_t const current_frame_id = ctx.framecount;

        auto const& culling_frustum = pipe.current_viewstate().frustum;

        std::vector<gua::node::Node*> visibility_persistence_vector;

        //going over all occlusion culling group nodes (currently only 1)
        for(auto const& occlusion_group_node : sorted_occlusion_group_nodes->second)
        {
            
            //push the root to traversal queue
            auto node_distance_pair_to_insert = std::make_pair(occlusion_group_node,
                                                scm::math::length_sqr(world_space_cam_pos - (find_raycast_intersection(occlusion_group_node, world_space_cam_pos) ) ) );

            traversal_priority_queue.push(node_distance_pair_to_insert);

            set_last_visibility_check_frame_id(occlusion_group_node->unique_node_id(), current_cam_node.uuid, current_frame_id);

            visibility_setting_queue.push(occlusion_group_node);

            while(!traversal_priority_queue.empty() || !query_queue.empty() )
            {
               
                while(!query_queue.empty()) {


                    if(ctx.render_context->query_result_available(query_queue.front().occlusion_query_pointer)) {


                        auto front_query_obj_queue = query_queue.front().occlusion_query_pointer;
                        auto front_query_vector = query_queue.front().nodes_to_query;
                        query_queue.pop();
                        ctx.render_context->collect_query_results(front_query_obj_queue);
                        uint64_t query_result = front_query_obj_queue->result();

                        handle_returned_query(
					                            ctx, pipe, desc,
					                            render_target,
					                            current_material,
					                            current_shader,
					                            view_projection_matrix,
					                            current_rasterizer_state,
					                            world_space_cam_pos,
					                            traversal_priority_queue,
					                            current_cam_node.uuid,
					                            query_result,
					                            front_query_vector,
					                            query_queue,
					                            current_frame_id);

                    } else {

                        if (v_query_queue.size()>0) {
                            auto current_vnode = v_query_queue.front();
                            v_query_queue.pop();
                            std::vector<gua::node::Node*> single_node_to_query;
                            single_node_to_query.push_back(current_vnode);



                            issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid, single_node_to_query);
                        }

                    }

                }

                if (!traversal_priority_queue.empty()) {
                    

                    //pop traversal queue
#ifdef USE_PRIORITY_QUEUE
                    auto current_node = traversal_priority_queue.top().first;
#else
                    auto current_node = traversal_priority_queue.front().first;
#endif // USE_PRIORITY_QUEUE
                    traversal_priority_queue.pop();

                    if(culling_frustum.intersects(current_node->get_bounding_box())) {

                        bool was_visible = false;

                        LastVisibility temp_last_visibility = get_last_visibility_checked_result(current_node->unique_node_id());
                        if (temp_last_visibility.frame_id == current_frame_id-1) {
                            was_visible = temp_last_visibility.result;
                        }


                        bool query_reasonable = get_query_reasonable(current_node->unique_node_id());

                        
                        if(!was_visible) {
                            //query previously invisible node n
                            i_query_queue.push(current_node);


                            //1 is for inital frame. After that the max will always be the max from the last frame
                            if(i_query_queue.size() >= batch_size_multi_query) {

                                issue_multi_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid, i_query_queue);
                            }
                        } else {

                            //if n is a leaf and the query is reasonable (find out what is reasonable)


                            if((current_node->get_children().size() == 0) && (query_reasonable)) {
                                v_query_queue.push(current_node);

                            }

                            traverse_node(current_node,
                                          ctx, pipe, desc,
                                          render_target,
                                          current_material,
                                          current_shader,
                                          current_rasterizer_state,
                                          world_space_cam_pos,
                                          traversal_priority_queue,
                                          current_cam_node.uuid,
                                          current_frame_id);

                        }

                    }
                }

                if (traversal_priority_queue.empty()) {

                    issue_multi_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid,i_query_queue);
                }

            }


            while(!v_query_queue.empty()) {
                //issue remaining queries from v-queue
                auto current_node = v_query_queue.front();
                v_query_queue.pop();
                std::vector<gua::node::Node*> single_node_to_query;
                single_node_to_query.push_back(current_node);
                issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid, single_node_to_query);

            }


        std::cout<<"VQueue size "<<v_query_queue.size()<< std::endl;
          while(!visibility_setting_queue.empty()) {
                auto current_node = visibility_setting_queue.front();
                visibility_setting_queue.pop();
                bool visibility_current_node = get_visibility(current_node->unique_node_id(), current_cam_node.uuid);
                set_last_visibility_checked_result(current_node->unique_node_id(), current_cam_node.uuid, current_frame_id, visibility_current_node);
                set_visibility_persistence(current_node->unique_node_id(), visibility_current_node);
                for (auto const& child : current_node->get_children()) {
                    visibility_setting_queue.push(child.get());
                }
            }

     
        }

        unbind_and_reset(ctx, render_target);

    }

}


//getter and setter
////////////////////////////////////////////////////////////////////////////////////////
int32_t OcclusionCullingAwareRenderer::get_last_visibility_check_frame_id(std::size_t in_unique_node_id, std::size_t in_camera_uuid) const {
    return last_visibility_check_frame_id_[in_unique_node_id][in_camera_uuid];
}

uint32_t OcclusionCullingAwareRenderer::get_visibility_persistence(std::size_t node_uuid) {
    VisiblityPersistence temp_vis_persistence = node_visibility_persistence[node_uuid];
    return temp_vis_persistence.persistence;
}

bool OcclusionCullingAwareRenderer::get_visibility(std::size_t in_unique_node_id, std::size_t in_camera_uuid) const {
    return is_visible_for_camera_[in_unique_node_id][in_camera_uuid];
}

LastVisibility OcclusionCullingAwareRenderer::get_last_visibility_checked_result(std::size_t in_unique_node_id) const {
    return last_visibility_checked_result_[in_unique_node_id];
}

bool OcclusionCullingAwareRenderer::get_query_reasonable(std::size_t node_uuid) const{
    return node_visibility_persistence[node_uuid].query_reasonable;
}

void OcclusionCullingAwareRenderer::set_visibility_persistence(std::size_t node_uuid, bool visibility) {
    VisiblityPersistence temp_vis_persistence = node_visibility_persistence[node_uuid];
//this part is for randomized queries of visible nodes


    //if the node just got visible
    if( !temp_vis_persistence.last_visibility && visibility) {
        //the paper suggested a random value between 5-10 so we will use mod 77 for now to randomize tests for visible nodes
        node_visibility_persistence[node_uuid].randomizer = std::rand() % 8;
        node_visibility_persistence[node_uuid].query_reasonable = false;
    }


    //if the node was previously visible and stays visible
    if(temp_vis_persistence.last_visibility && visibility) {
        if (node_visibility_persistence[node_uuid].randomizer < 1) {
            node_visibility_persistence[node_uuid].query_reasonable = true;
            node_visibility_persistence[node_uuid].randomizer = std::rand() % 8;
        } else {
            node_visibility_persistence[node_uuid].randomizer -= 1;
        }
    }


//this part is for actally setting the visibility persistence
    if (temp_vis_persistence.last_visibility == visibility) {
        node_visibility_persistence[node_uuid].persistence += 1;
    } else {
        node_visibility_persistence[node_uuid].persistence = 0;
    }

    node_visibility_persistence[node_uuid].last_visibility= visibility;


}

void OcclusionCullingAwareRenderer::set_visibility(std::size_t in_unique_node_id, std::size_t in_camera_uuid, bool is_visible) {
    is_visible_for_camera_[in_unique_node_id][in_camera_uuid] = is_visible;
}

void OcclusionCullingAwareRenderer::set_last_visibility_check_frame_id(std::size_t in_unique_node_id, std::size_t in_camera_uuid, int32_t current_frame_id) {
    last_visibility_check_frame_id_[in_unique_node_id][in_camera_uuid] = current_frame_id;
}


void OcclusionCullingAwareRenderer::set_geometry_visualisation_states(RenderContext const& ctx) {

    in_query_state_ = true;
    ctx.render_context->set_rasterizer_state(rs_cull_none_);

    ctx.render_context->apply_state_objects();
}


void OcclusionCullingAwareRenderer::set_occlusion_query_states(RenderContext const& ctx) {


    auto const& glapi = ctx.render_context->opengl_api();

    // we disable all color channels to save rasterization time
    glapi.glColorMask(false, false, false, false);

    // set depth state that tests, but does not write depth (otherwise we would have bounding box contours in the depth buffer -> not conservative anymore)
    ctx.render_context->set_rasterizer_state(rs_cull_none_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_test_without_writing_state_);
    ctx.render_context->apply_state_objects();
}


void OcclusionCullingAwareRenderer::set_last_visibility_checked_result(std::size_t in_unique_node_id, std::size_t in_camera_uuid, int32_t current_frame_id, bool result) {
    LastVisibility temp_last_visibility = LastVisibility{in_camera_uuid, current_frame_id, result};
    last_visibility_checked_result_[in_unique_node_id] = temp_last_visibility;
}



//CHC++ helper functions
////////////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingAwareRenderer::traverse_node(gua::node::Node* current_node,
        RenderContext const& ctx,
        Pipeline& pipe,
        PipelinePassDescription const& desc,
        RenderTarget& render_target,
        MaterialShader* current_material,
        std::shared_ptr<ShaderProgram> current_shader,
        scm::gl::rasterizer_state_ptr current_rasterizer_state,
        gua::math::vec3f const& world_space_cam_pos, std::priority_queue<std::pair<gua::node::Node*, double>,
        std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator >& traversal_priority_queue, std::size_t in_camera_uuid
        ,int64_t current_frame_id) {

    if((current_node->get_children().empty())) {

        
        
#ifdef RENDER_CHILD_CLASS

        if (in_query_state_) {
            auto const& glapi = ctx.render_context->opengl_api();
            glapi.glColorMask(true, true, true, true);
            ctx.render_context->set_depth_stencil_state(default_depth_test_);
            ctx.render_context->set_blend_state(default_blend_state_);
            ctx.render_context->apply();
            in_query_state_ = false;
        }

        RenderInfo current_render_info{current_material, current_shader, current_rasterizer_state};
        renderSingleNode(pipe, desc, current_node, current_render_info);
#else
        render_visible_leaf(current_node,
                            ctx, pipe,
                            render_target,
                            current_material,
                            current_shader,
                            current_rasterizer_state);
#endif


    } else {

        for (auto & child : current_node->get_children())
        {

            auto child_node_distance_pair_to_insert = std::make_pair(child.get(), scm::math::length_sqr(world_space_cam_pos - find_raycast_intersection(child.get(), world_space_cam_pos) ) );
            traversal_priority_queue.push(child_node_distance_pair_to_insert);
        }

    }
}

void OcclusionCullingAwareRenderer::pull_up_visibility(
    gua::node::Node* current_node,
    int64_t current_frame_id,
    std::size_t in_camera_uuid)
{


    auto temp_node = current_node;

    while(!get_visibility(temp_node->unique_node_id(), in_camera_uuid)) {

        set_visibility(temp_node->unique_node_id(), in_camera_uuid, true);

        if (temp_node->get_parent() != nullptr)
        {
            temp_node = temp_node->get_parent();
        }

    }

}

void OcclusionCullingAwareRenderer::issue_occlusion_query(RenderContext const& ctx, Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, std::queue<MultiQuery>& query_queue,
        int64_t current_frame_id, std::size_t in_camera_uuid,
        std::vector<gua::node::Node*> const& current_nodes) {

    auto current_node_id = current_nodes.front()->unique_node_id();
    auto occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);

    if(ctx.occlusion_query_objects.end() == occlusion_query_iterator ) {
        auto occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_SAMPLES_PASSED;
        if( OcclusionQueryType::Any_Samples_Passed == desc.get_occlusion_query_type() ) {
            occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_ANY_SAMPLES_PASSED;
        }
        ctx.occlusion_query_objects.insert(std::make_pair(current_node_id, ctx.render_device->create_occlusion_query(occlusion_query_mode) ) );

        occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);
    }

    bool fallback = false;
    auto current_shader = occlusion_query_array_box_program_;

    if (fallback)
    {
        current_shader = occlusion_query_box_program_;
    } else {
        current_shader = occlusion_query_array_box_program_;
    }

    current_shader->use(ctx);
    //auto vp_mat = view_projection_matrix;
    //current_shader->apply_uniform(ctx, "view_projection_matrix", math::mat4f(vp_mat));


    

    if(occlusion_culling_geometry_visualization_) {
        set_geometry_visualisation_states(ctx);
    } else {
        if (!in_query_state_) {
            set_occlusion_query_states(ctx);
            in_query_state_ = true;
        }
    }

  


    ctx.render_context->begin_query(occlusion_query_iterator->second);


    for (auto const& original_query_node : current_nodes)
    {
        if (fallback || original_query_node->get_children().empty())
        {
            // original draw call
            auto world_space_bounding_box = original_query_node->get_bounding_box();

            current_shader->set_uniform(ctx, scm::math::vec3f(world_space_bounding_box.min), "world_space_bb_min");
            current_shader->set_uniform(ctx, scm::math::vec3f(world_space_bounding_box.max), "world_space_bb_max");


            set_last_visibility_check_frame_id(original_query_node->unique_node_id(), in_camera_uuid, current_frame_id);


            auto const& glapi = ctx.render_context->opengl_api();


            ctx.render_context->apply();
            scm::gl::context_vertex_input_guard vig(ctx.render_context);

            ctx.render_context->bind_vertex_array(empty_vao_layout_);
            ctx.render_context->apply_vertex_input();
            glapi.glDrawArraysInstanced(GL_TRIANGLES, 0, 14, 1);


        } else {

            find_tightest_bounding_volume(original_query_node,
                                          ctx,
                                          current_shader,
                                          in_camera_uuid,
                                          current_frame_id, 3, 1.4f);
        }

    }

    ctx.render_context->end_query(occlusion_query_iterator->second);
    MultiQuery temp_multi_query = MultiQuery{occlusion_query_iterator->second, current_nodes};
    query_queue.push(temp_multi_query);

}

void OcclusionCullingAwareRenderer::issue_multi_query(RenderContext const& ctx, Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, std::queue<MultiQuery>& query_queue,
        int64_t current_frame_id, std::size_t in_camera_uuid, std::queue<gua::node::Node*>& i_query_queue) {

#ifdef DYNAMIC_BATCH_SIZE
    //calculate multi query size
    /*first sorte nodes in descending order based on probability of staying invisible --> number of frames in same vis state times array 
    if node in same state for more than 16 frames--> max */

    std::priority_queue<std::pair<gua::node::Node*, double>,
            std::vector<std::pair<gua::node::Node*, double> >, NodeVisibilityProbabilityPairComparator > visibility_persistence_probability;



    while(!i_query_queue.empty()) {
        auto node = i_query_queue.front();
        i_query_queue.pop();


        double visibility_persistence = get_visibility_persistence(node->unique_node_id());
        double keep;
        if (visibility_persistence > 15) {
            keep = keep_probability[15];
        } else {
            keep = keep_probability[visibility_persistence];
        }
        visibility_persistence_probability.push(std::make_pair(node, keep));

    }


    std::vector<gua::node::Node*> query_vector;

    float fail = 1;
    int number_of_nodes = 0;
    int max = 0;
    while(!visibility_persistence_probability.empty()) {
        auto node = visibility_persistence_probability.top();
        number_of_nodes++;
        fail *= node.second;
        float cost = (1+(1-fail)*number_of_nodes);
        float value = number_of_nodes/cost;
        if (value > max) {
            max = value;
            visibility_persistence_probability.pop();
            query_vector.push_back(node.first);
        } else {
            issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, in_camera_uuid, query_vector);
            max = 0;
            number_of_nodes = 0;
            fail = 1;
            query_vector.clear();
            batch_size_multi_query = std::max(20,number_of_nodes);

        }
        if (visibility_persistence_probability.empty()) {
            issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, in_camera_uuid, query_vector);
        }
        
    }

#else 

    uint64_t batch_size_max = 20;
    
    while(!i_query_queue.empty()) {

        uint64_t num_nodes_to_render = std::min(batch_size_max, i_query_queue.size() );
        //uint i = 0;
        std::vector<gua::node::Node*> temp_multi_query_vector;

        for(uint64_t i = 0; i < num_nodes_to_render; ++i) {
            //while(i <= batch_size_max) {
            auto node = i_query_queue.front();
            temp_multi_query_vector.push_back(node);
            i_query_queue.pop();
            ++i;
            //}
        }
        issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, in_camera_uuid, temp_multi_query_vector);

    }
#endif

}


void OcclusionCullingAwareRenderer::handle_returned_query(
 	RenderContext const& ctx,
    Pipeline& pipe,
    PipelinePassDescription const& desc,
    RenderTarget& render_target,
    MaterialShader* current_material,
    std::shared_ptr<ShaderProgram> current_shader,
    scm::math::mat4d const& view_projection_matrix,
    scm::gl::rasterizer_state_ptr current_rasterizer_state,
    gua::math::vec3f const& world_space_cam_pos,
    std::priority_queue<std::pair<gua::node::Node*, double>, std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator >& traversal_priority_queue,
    std::size_t in_camera_uuid,
    uint64_t query_result,
    std::vector<gua::node::Node*> front_query_vector,
    std::queue<MultiQuery>& query_queue,
    int64_t current_frame_id)
{
 
    unsigned int threshold = 0;
    switch( desc.get_occlusion_query_type() ) {
    case OcclusionQueryType::Number_Of_Samples_Passed:
        threshold = desc.get_occlusion_culling_fragment_threshold();
        break;

    //conservative approach. If any passed we render
    case OcclusionQueryType::Any_Samples_Passed:
        threshold = 0;
        break;

    default:
        Logger::LOG_WARNING << "OcclusionCullingTriMeshPass:: unknown occlusion query type encountered." << std::endl;
        break;
    }


    if(query_result>threshold) {

        if(front_query_vector.size()>1) { //this means our multi query failed.
            for (auto const& node : front_query_vector) {
                set_visibility(node->unique_node_id(), in_camera_uuid, true);
                std::vector<gua::node::Node*> single_node_to_query;
                single_node_to_query.push_back(node);
                issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, in_camera_uuid, single_node_to_query);

            }
        } else {

            for (auto const& current_node : front_query_vector) {



                bool was_visible = false;


                LastVisibility temp_last_visibility = get_last_visibility_checked_result(current_node->unique_node_id());
                if (temp_last_visibility.frame_id == current_frame_id-1) {
                    was_visible = temp_last_visibility.result;
                }
                set_visibility(current_node->unique_node_id(), in_camera_uuid, true);
                

                if(!was_visible) {

                    traverse_node(current_node,
                                  ctx, pipe, desc, 
                                  render_target,
                                  current_material,
                                  current_shader,
                                  current_rasterizer_state,
                                  world_space_cam_pos,
                                  traversal_priority_queue,
                                  in_camera_uuid,
                                  current_frame_id);

                }
                pull_up_visibility(current_node, current_frame_id, in_camera_uuid);
            }

        }

    } else {

        for (auto const& current_node : front_query_vector) {
            set_visibility(current_node->unique_node_id(), in_camera_uuid, false);
        }
    }
}



////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingAwareRenderer::render_visible_leaf(
    gua::node::Node* current_query_node,
    RenderContext const& ctx,
    Pipeline& pipe,
    RenderTarget& render_target,
    MaterialShader* current_material,
    std::shared_ptr<ShaderProgram> current_shader,
    scm::gl::rasterizer_state_ptr current_rasterizer_state)
{




    if (in_query_state_ == true ) {
        auto const& glapi = ctx.render_context->opengl_api();
        glapi.glColorMask(true, true, true, true);
        ctx.render_context->set_depth_stencil_state(default_depth_test_);
        ctx.render_context->set_blend_state(default_blend_state_);
        ctx.render_context->apply();
        in_query_state_ = false;
 
    }



    //make sure that we currently have a trimesh node in our hands
    if(std::type_index(typeid(node::TriMeshNode)) == std::type_index(typeid(*current_query_node)) ) {

        //std::cout << "ASSUMING THAT " << current_node->get_name() << " is a trimeshnode" << std::endl;
        auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(current_query_node));


        if(!depth_complexity_visualization_) {
           switch_state_based_on_node_material(ctx, tri_mesh_node, current_shader, current_material, render_target,
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
    

}

void OcclusionCullingAwareRenderer::find_tightest_bounding_volume(
    gua::node::Node* queried_node,
    RenderContext const& ctx,
    std::shared_ptr<ShaderProgram>& current_shader,
    size_t in_camera_uuid,
    size_t current_frame_id,
    unsigned int const dmax,
    float const smax) {
 std::vector<gua::node::Node*> tightest_nodes_vector;

    // if the node is interior we search the tighter bounding volume through its children
    std::queue<std::pair<uint32_t, std::vector<gua::node::Node*> > > depth_node_vector_queue;

    // max depth we search down
    depth_node_vector_queue.push({0, {queried_node}});

    // do the surface area check per level and find which level is the tightest
    while (!depth_node_vector_queue.empty()) {

        auto depth_node_vector_pair = depth_node_vector_queue.front();

        // pop this node from the queue and later repeat with the new deeper pair
        depth_node_vector_queue.pop();


        bool children_are_tighter = check_children_surface_area(depth_node_vector_pair.second, smax);


        // check if if the children level is tighter than the current level
        if ( children_are_tighter && depth_node_vector_pair.first < dmax)
        {
            std::vector<gua::node::Node*> checked_nodes_vector;

            // look through the vector
            for (auto const& parent_node : depth_node_vector_pair.second)
            {
                // if the node is leaf it is already the tightest node
                if (parent_node->get_children().empty()) {
                    tightest_nodes_vector.push_back(parent_node);
                }

                // if the node is interior push the children to the vector which later will be used to check their tightness
                else {

                    for (auto const& child_node : parent_node->get_children()) {
                        checked_nodes_vector.push_back(child_node.get());
                    }
                }

            }


            // if the vector is empty it means the all the tighter node is leaf and thus is already queried
            if (!checked_nodes_vector.empty())
            {
                // push the vector containing interior nodes to the queue for the next iteration
                depth_node_vector_queue.push({depth_node_vector_pair.first + 1, checked_nodes_vector});
            }
        }

        // the current level is the tightest  or comprises of the nodes with depth = 3 or leaf nodes
        else {

            tightest_nodes_vector.insert(tightest_nodes_vector.end(), depth_node_vector_pair.second.begin(), depth_node_vector_pair.second.end() );
        }
    }

    instanced_array_draw(tightest_nodes_vector, ctx, current_shader, in_camera_uuid, current_frame_id);


}

bool OcclusionCullingAwareRenderer::check_children_surface_area(
    std::vector<gua::node::Node*> const& in_parent_nodes,
    float const smax) const
{

    float parent_surface_area = 0.0f;
    float children_surface_area = 0.0f;

    for(auto const& parent_node: in_parent_nodes) {

        // sum of the surface area of every node in the vactor
        parent_surface_area += parent_node->get_bounding_box().surface_area();

        auto children_vector = parent_node->get_children();

        if (!children_vector.empty())
        {
            for (auto const& child: children_vector)
            {
                // sum of the surface area of every children node of every parent node in the vactor
                children_surface_area += child->get_bounding_box().surface_area();
            }
        }


        else {
            // what if there are both leaf and interior nodes in the in_parent vector????
            // include the leaf
            children_surface_area += parent_node->get_bounding_box().surface_area();
        }
    }


    bool is_tighter = (children_surface_area )  <= (parent_surface_area * smax);

    return is_tighter;

}


gua::math::vec3f OcclusionCullingAwareRenderer::find_raycast_intersection(gua::node::Node* node , gua::math::vec3f const& world_space_cam_pos) const {

    gua::math::vec3f const bb_min = gua::math::vec3f(node->get_bounding_box().min);

    gua::math::vec3f const bb_max = gua::math::vec3f(node->get_bounding_box().max);


    gua::math::vec3f const bb_mid_point = (bb_max + bb_min) / 2.0f;
    gua::math::vec3f const ray_vector = bb_mid_point - world_space_cam_pos;

    BoundingBoxSide const front_side = BoundingBoxSide{ gua::math::vec3f{bb_min.x,bb_min.y,bb_min.z},
                                                        gua::math::vec3f{bb_max.x,bb_max.y,bb_min.z},
                                                        2 };

    BoundingBoxSide const back_side = BoundingBoxSide{ gua::math::vec3f{bb_min.x,bb_min.y,bb_max.z},
                                                       gua::math::vec3f{bb_max.x,bb_max.y,bb_max.z},
                                                        2 
                                                      };

    BoundingBoxSide const left_side = BoundingBoxSide{ gua::math::vec3f{bb_min.x,bb_min.y,bb_min.z},
                                                       gua::math::vec3f{bb_min.x,bb_max.y,bb_max.z},
                                                       0
                                                      };

    BoundingBoxSide const right_side = BoundingBoxSide{ gua::math::vec3f{bb_max.x,bb_min.y,bb_min.z},
                                                        gua::math::vec3f{bb_max.x,bb_max.y,bb_max.z},
                                                       0
                                                       };

    BoundingBoxSide const bottom_side = BoundingBoxSide{ gua::math::vec3f{bb_min.x,bb_min.y,bb_min.z},
                                                         gua::math::vec3f{bb_max.x,bb_min.y,bb_max.z},
                                                         1
                                                       };
    BoundingBoxSide const top_side =  BoundingBoxSide{  gua::math::vec3f{bb_min.x,bb_max.y,bb_min.z},
                                                        gua::math::vec3f{bb_max.x,bb_max.y,bb_max.z},
                                                        1
                                                      };

    float const front_intersect_coof = (bb_min.z - world_space_cam_pos.z) / ray_vector.z;

    float const back_intersect_coof = (bb_max.z - world_space_cam_pos.z) / ray_vector.z;

    float const left_intersect_coof = (bb_min.x  - world_space_cam_pos.x) / ray_vector.x;

    float const right_intersect_coof = (bb_max.x - world_space_cam_pos.x) / ray_vector.x;

    float const bottom_intersect_coof = (bb_min.y - world_space_cam_pos.y) / ray_vector.y;

    float const top_intersect_coof = (bb_max.y - world_space_cam_pos.y) / ray_vector.y;

    std::pair<float, BoundingBoxSide> front_pair(front_intersect_coof, front_side);
    std::pair<float, BoundingBoxSide> back_pair(back_intersect_coof, back_side);
    std::pair<float, BoundingBoxSide> left_pair(left_intersect_coof, left_side);
    std::pair<float, BoundingBoxSide> right_pair(right_intersect_coof, right_side);
    std::pair<float, BoundingBoxSide> bottom_pair(bottom_intersect_coof, bottom_side);
    std::pair<float, BoundingBoxSide> top_pair(top_intersect_coof, top_side);


    std::vector<std::pair<float, BoundingBoxSide>> coofs_vector = {front_pair,back_pair,left_pair,right_pair,bottom_pair,top_pair};



    std::sort(coofs_vector.begin(),coofs_vector.end(), [](std::pair<float, BoundingBoxSide> a, std::pair<float, BoundingBoxSide> b) {
        return a.first < b.first;
    });

    gua::math::vec3f intersection_pt = gua::math::vec3f{std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max() };

    for (auto coof: coofs_vector)
    {
        if (coof.first >= 0.0f)
        {
            auto temp = coof.first * ray_vector + world_space_cam_pos;

            if (is_inside(temp, coof.second))
            {
                intersection_pt = temp;
                break;
            }

        }
    }

    if( intersection_pt.x == std::numeric_limits<float>::max() ) {
        std::cout << "INTERSECTION INVALID " << std::endl;
    }

    return intersection_pt;
}


bool OcclusionCullingAwareRenderer::is_inside(gua::math::vec3f const& intersection_pt, BoundingBoxSide const& bounding_plane) const {

    gua::math::vec3f min_pt = gua::math::vec3f(std::min(bounding_plane.min.x, bounding_plane.max.x),
                                               std::min(bounding_plane.min.y, bounding_plane.max.y),
                                               std::min(bounding_plane.min.z, bounding_plane.max.z));

    gua::math::vec3f max_pt = gua::math::vec3f(std::max(bounding_plane.min.x, bounding_plane.max.x),
                                               std::max(bounding_plane.min.y, bounding_plane.max.y),
                                               std::max(bounding_plane.min.z, bounding_plane.max.z));


    bool x_bounded = intersection_pt.x >= min_pt.x && intersection_pt.x <= max_pt.x;
    bool y_bounded = intersection_pt.y >= min_pt.y && intersection_pt.y <= max_pt.y;
    bool z_bounded = intersection_pt.z >= min_pt.z && intersection_pt.z <= max_pt.z;

    bool is_side_valid = true;

    if(bounding_plane.constant_axis != 0) {
        is_side_valid &= x_bounded;
    }
    if(bounding_plane.constant_axis != 1) {
        is_side_valid &= y_bounded;
    }
    if(bounding_plane.constant_axis != 2) {
        is_side_valid &= z_bounded;
    }

    return is_side_valid;

}

//Rendering related
////////////////////////////////////////////////////////////////////////////////////////




void OcclusionCullingAwareRenderer::instanced_array_draw(
    std::vector<gua::node::Node*> const& leaf_node_vector,
    RenderContext const& ctx,
    std::shared_ptr<ShaderProgram>& current_shader,
    size_t in_camera_uuid,
    size_t current_frame_id)
{

    uint32_t current_instance_ID = 0;


    for (auto const& leaf_node : leaf_node_vector)
    {

        auto world_space_bounding_box = leaf_node->get_bounding_box();


        std::string const uniform_string_bb_min = "world_space_bb_min";
        std::string const uniform_string_bb_max = "world_space_bb_max";


        math::vec3f bb_min_vec3f = math::vec3f(world_space_bounding_box.min);
        math::vec3f bb_max_vec3f = math::vec3f(world_space_bounding_box.max);

        current_shader->set_uniform(ctx, bb_min_vec3f, uniform_string_bb_min, current_instance_ID);
        current_shader->set_uniform(ctx, bb_max_vec3f, uniform_string_bb_max, current_instance_ID);

        set_last_visibility_check_frame_id(leaf_node->unique_node_id(), in_camera_uuid, current_frame_id);

        ++current_instance_ID;
    }


    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(empty_vao_layout_);
    ctx.render_context->apply_vertex_input();

    ctx.render_context->apply();

    auto const& glapi = ctx.render_context->opengl_api();
    // std::cout<< "RENDERING INSTANCES: " << current_instance_ID << std::endl;
    glapi.glDrawArraysInstanced(GL_TRIANGLE_STRIP, 0, 14, current_instance_ID);

}





void OcclusionCullingAwareRenderer::unbind_and_reset(RenderContext const& ctx, RenderTarget& render_target) {
    render_target.unbind(ctx);
    in_query_state_ = false;

    auto const& glapi = ctx.render_context->opengl_api();
    glapi.glColorMask(true, true, true, true);
    ctx.render_context->set_depth_stencil_state(default_depth_test_);
    ctx.render_context->set_blend_state(default_blend_state_);
    ctx.render_context->apply();

    ctx.render_context->reset_state_objects();
    ctx.render_context->sync();
}


void OcclusionCullingAwareRenderer::upload_uniforms_for_node(
    RenderContext const& ctx,
    node::TriMeshNode* tri_mesh_node,
    std::shared_ptr<ShaderProgram>& current_shader,
    Pipeline& pipe,
    scm::gl::rasterizer_state_ptr& current_rasterizer_state)
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

    //ctx.render_context->set_rasterizer_state(rs_cull_back_ );
    //ctx.render_context->apply_state_objects();



    //ctx.render_context->apply();
    ctx.render_context->apply_program();
}

void OcclusionCullingAwareRenderer::switch_state_based_on_node_material(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader,
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

void OcclusionCullingAwareRenderer::switch_state_for_depth_complexity_vis(RenderContext const& ctx, std::shared_ptr<ShaderProgram>& current_shader) {

    current_shader = depth_complexity_vis_program_;
    current_shader->use(ctx);


    ctx.render_context->set_blend_state(color_accumulation_state_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_writing_without_test_state_);
    ctx.render_context->apply_state_objects();

}

}