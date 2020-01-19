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


//#define OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
#define CHC_pp

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



std::array<float, 16> keep_probability;
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
    occlusion_query_array_box_program_stages_(), occlusion_query_array_box_program_(nullptr), //only one shader that is independent of the actual node material

    global_substitution_map_(smap)
{

    for(int32_t used_frames_index = 0; used_frames_index < keep_probability.size(); ++used_frames_index) {
        keep_probability[used_frames_index] = 0.99 - 0.7 * std::exp(-used_frames_index);
        std::cout << "Keep probability if node was consistent for " << used_frames_index << " frames: " << keep_probability[used_frames_index] << std::endl;
    }

// define all our shader sources for our 3 different shader programs we need/want to use here
// 1
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

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{


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
        render_CHC(pipe, desc, view_projection_matrix, world_space_cam_pos_euclidean);
        break;
    }

    case OcclusionCullingStrategy::Coherent_Hierarchical_Culling_PlusPlus: {
        render_CHC_plusplus(pipe, desc, view_projection_matrix, world_space_cam_pos_euclidean);
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
    auto type_sorted_tri_mesh_node_ptrs_iterator(scene.nodes.find(std::type_index(typeid(node::TriMeshNode)))); //We recevie a vector with only tri mesh node-type of scene

    //schauen ob tri-mesh node-typ in der map existiert (?) && ob der Vector enthaelt
    if(type_sorted_tri_mesh_node_ptrs_iterator != scene.nodes.end() && !type_sorted_tri_mesh_node_ptrs_iterator->second.empty())
    {
        //binding G-Buffer and camera
        RenderTarget& render_target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;



        bool write_depth = true;
        render_target.bind(ctx, write_depth);
        render_target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());


        // currently not needed - in case we make effects with different cameras
        //int view_id(camera.config.get_view_id());

        MaterialShader* current_material(nullptr); //set current material to null for first comparison
        std::shared_ptr<ShaderProgram> current_shader;
        auto current_rasterizer_state = rs_cull_back_; //backface culling
        ctx.render_context->apply();


        // loop through all objects, sorted by material ----------------------------
        //std::cout << "Num TriMeshNodes in Occlusion Pass: " << sorted_occlusion_group_nodes->second.size() << std::endl;

        //What is the reinterpret cast doing here? / if not reinterpret cast, the depth compl vis will not be found in pipeline pass (converts between types by reinterpreting underlying bit pattern)
        auto const occlusion_culling_pipeline_pass_description = reinterpret_cast<OcclusionCullingTriMeshPassDescription const*>(&desc);
        bool depth_complexity_vis = occlusion_culling_pipeline_pass_description->get_enable_depth_complexity_vis();



        std::vector<std::pair<gua::node::Node*, double> > node_distance_pair_vector;

        for(auto const& current_node_ptr : type_sorted_tri_mesh_node_ptrs_iterator->second) {
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

        uint object_render_count = 0;
        //iterate through sorted nodes
        for(auto const& object : node_distance_pair_vector)
        {
            auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object.first)); //backcasting to trimesh node
            if(pipe.current_viewstate().shadow_mode && tri_mesh_node->get_shadow_mode() == ShadowMode::OFF)
            {
                continue;
            }

            if(!tri_mesh_node->get_render_to_gbuffer()) //sometimes nodes should be invisible and we can get with this function
            {
                continue;
            }

            if(depth_complexity_vis) { // we render the scene normally if depth complexity visualisation is false.
                switch_state_for_depth_complexity_vis(ctx, current_shader); //rendering with depth complexity on
            } else {
                //We check if the material is the same as before and only then initiate state change-> updates current shader (reference) and material (pointer)
                switch_state_based_on_node_material(ctx, tri_mesh_node, current_shader, current_material, render_target,
                                                    pipe.current_viewstate().shadow_mode, pipe.current_viewstate().camera.uuid);

            }

            //wenn wir einen shader haben (kein nullptr) und die tri-mesh-node nicht leer ist
            if(current_shader && tri_mesh_node->get_geometry())
            {
                //setting backface, wireframe and normals. current shader as reference
                upload_uniforms_for_node(ctx, tri_mesh_node, current_shader, pipe, current_rasterizer_state);

                tri_mesh_node->get_geometry()->draw(pipe.get_context()); //Here we draw!!!

                ++object_render_count;
            }
        }

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
        std::cout << "Rendered " << object_render_count << "/" << type_sorted_tri_mesh_node_ptrs_iterator->second.size() << " objects" << std::endl;
#endif //OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE

        render_target.unbind(ctx);


        ctx.render_context->reset_state_objects();
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

    // Here we go in the map (of serialize scene) and look for all nodes from the Type TriMesh
    auto type_sorted_node_ptrs_iterator(scene.nodes.find(std::type_index(typeid(node::TriMeshNode))));



    // check if any nodes at all were serialized (nodes that are frustum culled would not appear here) !! Important !!
    if(type_sorted_node_ptrs_iterator != scene.nodes.end() && !type_sorted_node_ptrs_iterator->second.empty())
    {
        // the RenderTarget object target - binding GBuffer
        RenderTarget& render_target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

        //Sorting based on material to minimize state-changes
        //std::sort(type_sorted_node_ptrs_iterator->second.begin(), type_sorted_node_ptrs_iterator->second.end(), [](node::Node* a, node::Node* b) {
        //    return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
        //});


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

void OcclusionCullingTriMeshRenderer::render_CHC(Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {

#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
    std::cout << "Start of new Frame " << std::endl;
#endif
    RenderContext const& ctx(pipe.get_context());
    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));


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
            // creates occlusion query objects
            // add all to query queue

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
        for(auto const& occlusion_group_node : sorted_occlusion_group_nodes->second)
        {

            // use a queue for breadth first search (stack would do breadth first search)
            int rendered_nodes = 0;
            int total_num_trimesh_nodes = 0;

            // add root node of our occlusion hierarchy to the traversal queue
            auto node_distance_pair_to_insert = std::make_pair(occlusion_group_node,
                                                scm::math::length_sqr(world_space_cam_pos - (occlusion_group_node->get_bounding_box().max + occlusion_group_node->get_bounding_box().min)/2.0f ) );

            traversal_priority_queue.push(node_distance_pair_to_insert);

            while(!traversal_priority_queue.empty() || !i_query_queue.empty() )
            {

                //------------------------------------------------------------------PART 1
                bool result_available = false;
                if (!i_query_queue.empty()) {
                    result_available = ctx.render_context->query_result_available(i_query_queue.front().second);
                }

                while(!i_query_queue.empty() && (result_available || traversal_priority_queue.empty()))
                {

                    auto current_query_node = i_query_queue.front().first;
                    auto front_query_obj_queue = i_query_queue.front().second;

                    i_query_queue.pop();

                    /*
                    std::cout<<"beginning query queue :"<< " current node name " << current_query_node->get_name() <<
                    " is visible? " << get_visibility(current_query_node->unique_node_id(), current_cam_node.uuid)<<std::endl;
                    */

                    // waiting for the result of the previously invisible node
                    while(!result_available)
                    {
                        result_available = ctx.render_context->query_result_available(front_query_obj_queue);
                    }

                    /*
                    std::cout<<"after waiting :"<< " current node name " << current_query_node->get_name() <<
                    " is visible? " << get_visibility(current_query_node->unique_node_id(), current_cam_node.uuid)<<std::endl;
                    */

                    ctx.render_context->collect_query_results(front_query_obj_queue);

                    //int64_t last_checked_frame_id_for_node = get_last_visibility_check_frame_id(current_query_node->unique_node_id(), current_cam_node.uuid);

                    // the result contains the number of sampled that were created (in mode OQMODE_SAMPLES_PASSED) or 0 or 1 (in mode OQMODE_ANY_SAMPLES_PASSED)
                    uint64_t query_result = front_query_obj_queue->result();


                    if (current_query_node->get_children().empty()) {
                        ++total_num_trimesh_nodes;
                    }


                    bool render_current_node = false;


                    switch( desc.get_occlusion_query_type() ) {
                    case OcclusionQueryType::Number_Of_Samples_Passed:
                        //So if we define a certain threshold, then check if the number of returned fragments is higher than threshold, only then render (not conservative?)
                        if(query_result > desc.get_occlusion_culling_fragment_threshold()) {
                            render_current_node = true;
                        } else {
                            render_current_node = false;
                        }
                        break;

                    //conservative approach. If any passed we render
                    case OcclusionQueryType::Any_Samples_Passed:
                        if(query_result > 0) {
                            render_current_node = true;
                        } else {
                            render_current_node = false;
                        }
                        break;

                    default:
                        Logger::LOG_WARNING << "OcclusionCullingTriMeshPass:: unknown occlusion query type encountered." << std::endl;
                        break;
                    }

                    if (render_current_node) //query_result > 0 /*desc.get_occlusion_culling_fragment_threshold()*/ )
                    {
                        // pull up visibility
                        pull_up_visibility(current_query_node, current_cam_node.uuid);

                        // For interior Nodes
                        if (!current_query_node->get_children().empty())
                        {
                            // Traverse the interior Nodes
                            for(auto& child: current_query_node->get_children()) {

                                auto child_node_distance_pair_to_insert = std::make_pair(child.get(), scm::math::length_sqr(world_space_cam_pos - (child->get_bounding_box().max + child->get_bounding_box().min)/2.0f ) );
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
                    /*
                    std::cout<<"end query_queue :"<< " current node name " << current_query_node->get_name() <<
                    " is visible? " << get_visibility(current_query_node->unique_node_id(), current_cam_node.uuid)<<std::endl;
                    */
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
                                                + current_node->get_bounding_box().min)/2.0f ) ;
#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
                    std::cout << "Traversing node: " << current_node->get_name() << "  with dist to cam: " << distance_cam_to_node << std::endl;
#endif

                    bool is_in_frustum = culling_frustum.intersects(current_node->get_bounding_box());

                    if (is_in_frustum)
                    {

                        bool visibility_current_node = get_visibility(current_node->unique_node_id(), current_cam_node.uuid);

                        int32_t node_last_checked_frame_id = get_last_visibility_check_frame_id(current_node->unique_node_id(), current_cam_node.uuid);

                        bool was_visible = false;
                        if (visibility_current_node && (node_last_checked_frame_id == 0
                                                        || (node_last_checked_frame_id == current_frame_id -1) ) )
                        {
                            was_visible = true;
                        }

                        bool is_opened = false;
                        if (!current_node->get_children().empty() && was_visible) {
                            is_opened = true;
                        }

                        set_visibility(current_node->unique_node_id(), current_cam_node.uuid, false);
                        set_last_visibility_check_frame_id(current_node->unique_node_id(), current_cam_node.uuid, current_frame_id);

                        if (!is_opened)
                        {
                            auto current_node_id = current_node->unique_node_id();

                            /////// Check whether occlusion query objects for this node were already created
                            auto occlusion_query_iterator = ctx.occlusion_query_objects.find(current_node_id);

                            if(ctx.occlusion_query_objects.end() == occlusion_query_iterator ) {
                                // get occlusion query mode
                                //default: Number of Sampled Passed -> if 0 it is invisible?
                                auto occlusion_query_mode = scm::gl::occlusion_query_mode::OQMODE_SAMPLES_PASSED;


                                // change occlusion query type if we only want to know whether any fragment was visible
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
                                set_occlusion_query_states(ctx);
                            }

                            auto current_occlusion_query_object = occlusion_query_iterator->second;
                            // begin query?
                            ctx.render_context->begin_query(current_occlusion_query_object); //second is the query object from the map
                            //pipe knows context
                            pipe.draw_box(); // when we draw, the query is admitted for all drawn objects. in current_shader we create bb
                            ctx.render_context->end_query(current_occlusion_query_object);

                            if(!was_visible) {
                                i_query_queue.push(std::make_pair(current_node, current_occlusion_query_object) );
                            } else {
                                v_query_queue.push(std::make_pair(current_node, current_occlusion_query_object) );
                            }
                        }


                        if (was_visible)
                        {
                            if(!(current_node->get_children().empty())) {

                                // Traverse the nodes
                                for (auto & child : current_node->get_children())
                                {

                                    auto child_node_distance_pair_to_insert = std::make_pair(child.get(), scm::math::length_sqr(world_space_cam_pos - (child->get_bounding_box().max + child->get_bounding_box().min)/2.0f ) );
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
                        /*
                        std::cout<<"end of priority queue: "<< ctx.framecount <<" current node name " << current_node->get_name() <<
                        " is visible? " << get_visibility(current_node->unique_node_id(), current_cam_node.uuid)<<std::endl;
                        */
                    }
                }

            }

            while(!v_query_queue.empty()) {

                auto current_query_node = v_query_queue.front().first;
                auto front_query_obj_queue = v_query_queue.front().second;

                v_query_queue.pop();



/////////////////////////////////


                /*
                std::cout<<"beginning query queue :"<< " current node name " << current_query_node->get_name() <<
                " is visible? " << get_visibility(current_query_node->unique_node_id(), current_cam_node.uuid)<<std::endl;
                */
                // busy waiting - instead of doing something meaningful, we stall the CPU and therefore starve the GPU

                bool result_available = false;
                while(!result_available)
                {
                    result_available = ctx.render_context->query_result_available(front_query_obj_queue);
                }

                /*
                std::cout<<"after waiting :"<< " current node name " << current_query_node->get_name() <<
                " is visible? " << get_visibility(current_query_node->unique_node_id(), current_cam_node.uuid)<<std::endl;
                */

                ctx.render_context->collect_query_results(front_query_obj_queue);

                //int64_t last_checked_frame_id_for_node = get_last_visibility_check_frame_id(current_query_node->unique_node_id(), current_cam_node.uuid);

                // the result contains the number of sampled that were created (in mode OQMODE_SAMPLES_PASSED) or 0 or 1 (in mode OQMODE_ANY_SAMPLES_PASSED)
                uint64_t query_result = front_query_obj_queue->result();


                bool render_current_node = false;


                switch( desc.get_occlusion_query_type() ) {
                case OcclusionQueryType::Number_Of_Samples_Passed:
                    //So if we define a certain threshold, then check if the number of returned fragments is higher than threshold, only then render (not conservative?)
                    if(query_result > desc.get_occlusion_culling_fragment_threshold()) {
                        render_current_node = true;
                    } else {
                        render_current_node = false;
                    }
                    break;

                //conservative approach. If any passed we render
                case OcclusionQueryType::Any_Samples_Passed:
                    if(query_result > 0) {
                        render_current_node = true;
                    } else {
                        render_current_node = false;
                    }
                    break;

                default:
                    Logger::LOG_WARNING << "OcclusionCullingTriMeshPass:: unknown occlusion query type encountered." << std::endl;
                    break;
                }

                if (render_current_node) //query_result > 0 /*desc.get_occlusion_culling_fragment_threshold()*/ )
                {

                    set_visibility(current_query_node->unique_node_id(), current_cam_node.uuid, true);


                }
/////////////////////////////////
            }


//#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
            //std::cout<<" rendered nodes " << rendered_nodes << "/" << total_num_trimesh_nodes << std::endl;
//#endif
        }

        unbind_and_reset(ctx, render_target);

    }

}


void OcclusionCullingTriMeshRenderer::render_CHC_plusplus(Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, gua::math::vec3f const& world_space_cam_pos) {


#ifdef OCCLUSION_CULLING_TRIMESH_PASS_VERBOSE
    std::cout << "Start of new Frame " << std::endl;
#endif
    RenderContext const& ctx(pipe.get_context());
    auto& scene = *pipe.current_viewstate().scene;
    auto sorted_occlusion_group_nodes(scene.nodes.find(std::type_index(typeid(node::OcclusionCullingGroupNode))));

//if oc-group note existed in map of serialized scene and is not empty
    if(sorted_occlusion_group_nodes != scene.nodes.end() && !sorted_occlusion_group_nodes->second.empty())
    {
        auto& render_target = *pipe.current_viewstate().target;
        auto const& camera = pipe.current_viewstate().camera;

        bool write_depth = true;
        render_target.bind(ctx, write_depth);
        render_target.set_viewport(ctx);

        scm::math::vec2ui render_target_dims(render_target.get_width(), render_target.get_height());

        //int view_id(camera.config.get_view_id());
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

        //query queue - where it is used?
        std::queue<MultiQuery> query_queue;

        //query queue for nodes that were previously invisible and need to be queried
        std::queue<gua::node::Node*> i_query_queue;

        //query queue for nodes that were previously visible and need to be queried but can wait until the end of the frame
        std::queue<gua::node::Node*> v_query_queue;

        // reset visibility status if the node was never visited
        // only one element is in the vector. is it necessary to do a for loop here? Or for the case we have more than one oc-group_node
        for( auto& occlusion_group_node : sorted_occlusion_group_nodes->second )
        {

            int32_t last_visibility_check_frame_id = get_last_visibility_check_frame_id(occlusion_group_node->unique_node_id(), current_cam_node.uuid);
            // if we never checked set the visibility status for this node, it will be 0.
            // in this case, we recursively set the entire hierarchy to visible

            if( 0 != last_visibility_check_frame_id) {
                continue;
            }

            std::cout<<"last visited" <<last_visibility_check_frame_id<<std::endl;
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
#ifdef CHC_pp
        //std::cout<<"Using Plusplus"<<std::endl;
#endif

#ifdef USE_PRIORITY_QUEUE
        std::priority_queue<std::pair<gua::node::Node*, double>,
            std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator > traversal_priority_queue;
#else
        std::queue<std::pair<gua::node::Node*, double> > traversal_priority_queue;
#endif

        int64_t const current_frame_id = ctx.framecount;

        auto const& culling_frustum = pipe.current_viewstate().frustum;

        //going over all occlusion culling group nodes (currently only 1)
        for(auto const& occlusion_group_node : sorted_occlusion_group_nodes->second)
        {
            //push the root to traversal queue
            auto node_distance_pair_to_insert = std::make_pair(occlusion_group_node,
                                                scm::math::length_sqr(world_space_cam_pos - (occlusion_group_node->get_bounding_box().max + occlusion_group_node->get_bounding_box().min)/2.0f ) );

            traversal_priority_queue.push(node_distance_pair_to_insert);

            set_last_visibility_check_frame_id(occlusion_group_node->unique_node_id(), current_cam_node.uuid, current_frame_id);

            while(!traversal_priority_queue.empty() || !query_queue.empty() )
            {
                //std::cout<<"frame "<< current_frame_id<< ",Traversal Queue size"  << traversal_priority_queue.size() << ", Query Queue Size " << query_queue.size()<<std::endl;

                while(!query_queue.empty()) {

                    //if the first query is finished
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
                            depth_complexity_vis,
                            world_space_cam_pos,
                            traversal_priority_queue,
                            current_cam_node.uuid,
                            query_result,
                            front_query_vector,
                            query_queue,
                            current_frame_id);

                    } else {
                        //query next prev- visible node from v-queue
#ifdef CHC_pp
                        if (v_query_queue.size()>0) {

                            auto current_vnode = v_query_queue.front();
                            v_query_queue.pop();
                            std::vector<gua::node::Node*> single_node_to_query;
                            single_node_to_query.push_back(current_vnode);



                            issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid, single_node_to_query);
                        }
#endif
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

                        bool visibility_current_node = get_visibility(current_node->unique_node_id(), current_cam_node.uuid);

                        int32_t node_last_checked_frame_id = get_last_visibility_check_frame_id(current_node->unique_node_id(), current_cam_node.uuid);

                        bool was_visible = false;
                        if (visibility_current_node && (node_last_checked_frame_id == 0
                                                        || (node_last_checked_frame_id == current_frame_id -1) ) )
                        {
                            was_visible = true;
                        }


                        bool was_visible2 = false;

                        LastVisibility temp_last_visibility = get_last_visibility_checked_result(current_node->unique_node_id());
                        if (temp_last_visibility.frame_id == current_frame_id-1) {
                            was_visible = temp_last_visibility.result;
                        }

                        if(!was_visible2) {
                            //query previously invisible node n
                            i_query_queue.push(current_node);

                            /***************************placeholder number here! testing for better one***********************/
                            if(i_query_queue.size() >= 20) {
                                issue_multi_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid, i_query_queue);
                            }
                        } else {


                            //if n is a leaf and the query is reasonable (find out what is reasonable)

                            if(current_node->get_children().size() == 0 && 1==1) {
                                //push N to v-queue^s

#ifdef CHC_pp
                                v_query_queue.push(current_node);
#endif
                            }


                            traverse_node(current_node,
                                          ctx, pipe,
                                          render_target,
                                          current_material,
                                          current_shader,
                                          current_rasterizer_state,
                                          depth_complexity_vis,
                                          world_space_cam_pos,
                                          traversal_priority_queue,
                                          current_cam_node.uuid);

                        }
                    }
                }
#ifdef CHC_pp
                if (traversal_priority_queue.empty()) {
                    issue_multi_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid,i_query_queue);
                }
#endif
            }
#ifdef CHC_pp

            while(!v_query_queue.empty()) {
                //issue remaining queries from v-queue
                auto current_node = v_query_queue.front();
                v_query_queue.pop();
                std::vector<gua::node::Node*> single_node_to_query;
                single_node_to_query.push_back(current_node);
                issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, current_cam_node.uuid, single_node_to_query);

            }
#endif
        }

        unbind_and_reset(ctx, render_target);

    }

}

void OcclusionCullingTriMeshRenderer::handle_returned_query(
    RenderContext const& ctx,
    Pipeline& pipe,
    PipelinePassDescription const& desc,
    RenderTarget& render_target,
    MaterialShader* current_material,
    std::shared_ptr<ShaderProgram> current_shader,
    scm::math::mat4d const& view_projection_matrix,
    scm::gl::rasterizer_state_ptr current_rasterizer_state,
    bool& depth_complexity_vis,
    gua::math::vec3f const& world_space_cam_pos,
    std::priority_queue<std::pair<gua::node::Node*, double>, std::vector<std::pair<gua::node::Node*, double> >, NodeDistancePairComparator >& traversal_priority_queue,
    std::size_t in_camera_uuid,
    uint64_t query_result,
    std::vector<gua::node::Node*> front_query_vector,
    std::queue<MultiQuery>& query_queue,
    int64_t const current_frame_id)
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
#ifdef CHC_pp
        if(front_query_vector.size()>1) { //this means our multi query failed.
            for (auto const& node : front_query_vector) {
                set_last_visibility_checked_result(node->unique_node_id(), in_camera_uuid, current_frame_id, true);
                std::vector<gua::node::Node*> single_node_to_query;
                single_node_to_query.push_back(node);
                issue_occlusion_query(ctx, pipe, desc, view_projection_matrix, query_queue, current_frame_id, in_camera_uuid, single_node_to_query);

            }
        } else {
#endif
            for (auto const& current_node : front_query_vector) {
                //if node was not visible in last frame
                set_last_visibility_checked_result(current_node->unique_node_id(), in_camera_uuid, current_frame_id, true);

                bool visibility_current_node = get_visibility(current_node->unique_node_id(), in_camera_uuid);

                int32_t node_last_checked_frame_id = get_last_visibility_check_frame_id(current_node->unique_node_id(), in_camera_uuid);

                bool was_visible = false;
                if (visibility_current_node && (node_last_checked_frame_id == 0
                                                || (node_last_checked_frame_id == current_frame_id -1) ) )
                {
                    was_visible = true;
                }

                bool was_visible2 = false;

                LastVisibility temp_last_visibility = get_last_visibility_checked_result(current_node->unique_node_id());
                if (temp_last_visibility.frame_id == current_frame_id-1) {
                    was_visible = temp_last_visibility.result;
                }

                if(!was_visible2) {

                    traverse_node(current_node,
                                  ctx, pipe,
                                  render_target,
                                  current_material,
                                  current_shader,
                                  current_rasterizer_state,
                                  depth_complexity_vis,
                                  world_space_cam_pos,
                                  traversal_priority_queue,
                                  in_camera_uuid);
                }
                pull_up_visibility(current_node, current_frame_id, in_camera_uuid);
            }
#ifdef CHC_pp
        }

    } else {
#endif
        for (auto const& current_node : front_query_vector) {
            set_last_visibility_checked_result(current_node->unique_node_id(), in_camera_uuid, current_frame_id, false);
            set_visibility(current_node->unique_node_id(), in_camera_uuid, false);
        }
    }
}

void OcclusionCullingTriMeshRenderer::traverse_node(
    gua::node::Node* current_node,
    RenderContext const& ctx,
    Pipeline& pipe,
    RenderTarget& render_target,
    MaterialShader* current_material,
    std::shared_ptr<ShaderProgram> current_shader,
    scm::gl::rasterizer_state_ptr current_rasterizer_state,
    bool& depth_complexity_vis, gua::math::vec3f const& world_space_cam_pos,
    std::priority_queue<std::pair<gua::node::Node*, double>,
    std::vector<std::pair<gua::node::Node*, double> >,
    NodeDistancePairComparator >& traversal_priority_queue,
    std::size_t in_camera_uuid)
{

    if((current_node->get_children().empty())) {
        render_visible_leaf(current_node,
                            ctx, pipe,
                            render_target,
                            current_material,
                            current_shader,
                            current_rasterizer_state,
                            depth_complexity_vis);

    } else {

        for (auto & child : current_node->get_children())
        {
            auto child_node_distance_pair_to_insert = std::make_pair(child.get(), scm::math::length_sqr(world_space_cam_pos - (child->get_bounding_box().max + child->get_bounding_box().min)/2.0f ) );
            traversal_priority_queue.push(child_node_distance_pair_to_insert);
        }
        set_visibility(current_node->unique_node_id(), in_camera_uuid, false);
    }
}

void OcclusionCullingTriMeshRenderer::issue_occlusion_query(
    RenderContext const& ctx,
    Pipeline& pipe,
    PipelinePassDescription const& desc,
    scm::math::mat4d const& view_projection_matrix,
    std::queue<MultiQuery>& query_queue,
    int64_t current_frame_id,
    std::size_t in_camera_uuid,
    std::vector<gua::node::Node*> const& current_nodes)
{

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

    // for testing and comparison purpose
    bool fallback = false;

    auto current_shader = occlusion_query_array_box_program_;

    if (fallback)
    {
        current_shader = occlusion_query_box_program_;
    } else {
        current_shader = occlusion_query_array_box_program_;
    }


    current_shader->use(ctx);
    auto vp_mat = view_projection_matrix;
    current_shader->apply_uniform(ctx, "view_projection_matrix", math::mat4f(vp_mat));

    set_occlusion_query_states(ctx);
    ctx.render_context->begin_query(occlusion_query_iterator->second);


    for (auto const& original_query_node : current_nodes)
    {


        if (fallback)
        {
            // original draw call
            auto world_space_bounding_box = original_query_node->get_bounding_box();

            current_shader->set_uniform(ctx, scm::math::vec3f(world_space_bounding_box.min), "world_space_bb_min");
            current_shader->set_uniform(ctx, scm::math::vec3f(world_space_bounding_box.max), "world_space_bb_max");

            set_last_visibility_check_frame_id(original_query_node->unique_node_id(), in_camera_uuid, current_frame_id);

            //replacement for pipe.draw_box()
            ctx.render_context->apply();
            scm::gl::context_vertex_input_guard vig(ctx.render_context);

            ctx.render_context->bind_vertex_array(empty_vao_layout_);
            ctx.render_context->apply_vertex_input();

            auto const& glapi = ctx.render_context->opengl_api();
            glapi.glDrawArraysInstanced(GL_TRIANGLES, 0, 36, 1);

        } else {

            // vector of nodes that needs to be queried
            std::vector<gua::node::Node*> leaf_nodes_vector;

            // if the node is a leaf in hierachy we issue the query directly
            if( original_query_node->get_children().empty() ) {

                leaf_nodes_vector.push_back(original_query_node);

            } else { // if the node is interior we search the tighter bounding volume through its children

                std::queue<std::pair<uint32_t, std::vector<gua::node::Node*> > > depth_node_vector_queue;

                // max depth we search down
                unsigned int const dmax = 3;

                depth_node_vector_queue.push({0, {original_query_node}});

                // do the surface area check per level and find which level is the tightest
                while(!depth_node_vector_queue.empty()) {

                    auto checked_depth_node_vector_pair = depth_node_vector_queue.front();

                    // pop this node from the queue and later repeat with the new deeper pair
                    depth_node_vector_queue.pop();

                    // check if if the children level is tighter than the current level
                    if ( check_children_surface_area(checked_depth_node_vector_pair.second) && checked_depth_node_vector_pair.first < dmax)
                    {
                        std::vector<gua::node::Node*> checked_nodes_vector;

                        // look through the vector
                        for (auto const& checked_node: checked_depth_node_vector_pair.second)
                        {
                            // if the node is leaf issue the query
                            if(checked_node->get_children().empty()) {
                                leaf_nodes_vector.push_back(checked_node);
                            }

                            // if the node is interior push the children to the vector which later will be used to check their tightness
                            else {

                                for(auto const& child_of_checked_node: checked_node->get_children()) {
                                    checked_nodes_vector.push_back(child_of_checked_node.get());
                                }
                            }

                        }


                        // if the vector is empty it means the all the checked node is leaf and thus is already queried
                        if (!checked_nodes_vector.empty())
                        {
                            // push the vector containing interior nodes to the queue for the next iteration
                            depth_node_vector_queue.push({checked_depth_node_vector_pair.first+1, checked_nodes_vector});
                        }
                    }

                    // the current level is the tightest  or comprises of the nodes with depth = 3 or leaf nodes
                    else {
                        
                        leaf_nodes_vector.insert(leaf_nodes_vector.end(), checked_depth_node_vector_pair.second.begin(), checked_depth_node_vector_pair.second.end() );
                    }
                }

            }

            instanced_array_draw(leaf_nodes_vector, ctx, current_shader, in_camera_uuid, current_frame_id);

        }

    }

    ctx.render_context->end_query(occlusion_query_iterator->second);

    MultiQuery temp_multi_query = MultiQuery{occlusion_query_iterator->second, current_nodes};

    query_queue.push(temp_multi_query);

}

void OcclusionCullingTriMeshRenderer::issue_multi_query(RenderContext const& ctx, Pipeline& pipe, PipelinePassDescription const& desc,
        scm::math::mat4d const& view_projection_matrix, std::queue<MultiQuery>& query_queue,
        int64_t current_frame_id, std::size_t in_camera_uuid, std::queue<gua::node::Node*>& i_query_queue) {

    while(!i_query_queue.empty()) {
        uint64_t batch_size_max = 20; //only paceholder! has to be implemented correctly still!!!

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
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::upload_uniforms_for_node(
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

    ctx.render_context->set_rasterizer_state(rs_cull_back_ );
    ctx.render_context->apply_state_objects();

    ctx.render_context->apply_program();

    ctx.render_context->apply();

}



////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshRenderer::set_occlusion_query_states(RenderContext const& ctx) {
    auto const& glapi = ctx.render_context->opengl_api();

    // we disable all color channels to save rasterization time
    glapi.glColorMask(false, false, false, false);

    // set depth state that tests, but does not write depth (otherwise we would have bounding box contours in the depth buffer -> not conservative anymore)
    ctx.render_context->set_rasterizer_state(rs_cull_none_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_test_without_writing_state_);
    ctx.render_context->apply_state_objects();
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

    current_shader = depth_complexity_vis_program_;
    current_shader->use(ctx);


    ctx.render_context->set_blend_state(color_accumulation_state_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_writing_without_test_state_);
    ctx.render_context->apply_state_objects();

}

////////////////////////////////////////////////////////////////////////////////

bool OcclusionCullingTriMeshRenderer::get_visibility(std::size_t in_unique_node_id, std::size_t in_camera_uuid) const {
    return is_visible_for_camera_[in_unique_node_id][in_camera_uuid];
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::set_visibility(std::size_t in_unique_node_id, std::size_t in_camera_uuid, bool is_visible) {
    is_visible_for_camera_[in_unique_node_id][in_camera_uuid] = is_visible;
}

////////////////////////////////////////////////////////////////////////////////

int32_t OcclusionCullingTriMeshRenderer::get_last_visibility_check_frame_id(std::size_t in_unique_node_id, std::size_t in_camera_uuid) const {
    return last_visibility_check_frame_id_[in_unique_node_id][in_camera_uuid];
}

////////////////////////////////////////////////////////////////////////////////

void OcclusionCullingTriMeshRenderer::set_last_visibility_check_frame_id(std::size_t in_unique_node_id, std::size_t in_camera_uuid, int32_t current_frame_id) {
    last_visibility_check_frame_id_[in_unique_node_id][in_camera_uuid] = current_frame_id;
}

////////////////////////////////////////////////////////////////////////////////
LastVisibility OcclusionCullingTriMeshRenderer::get_last_visibility_checked_result(std::size_t in_unique_node_id) const {
    return last_visibility_checked_result_[in_unique_node_id];
}

////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshRenderer::set_last_visibility_checked_result(std::size_t in_unique_node_id, std::size_t in_camera_uuid, int32_t current_frame_id, bool result) {
    LastVisibility temp_last_visibility = LastVisibility{in_camera_uuid, current_frame_id, result};
    last_visibility_checked_result_[in_unique_node_id] = temp_last_visibility;
}

////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshRenderer::pull_up_visibility(
    gua::node::Node* current_node,
    std::size_t in_camera_uuid)
{

    // store the node pointer
    auto temp_node = current_node;
    //std::cout<< "current node name " << temp_node->get_name()<<std::endl;
    // pull up algorithm as stated by CHC paper
    while(!get_visibility(temp_node->unique_node_id(), in_camera_uuid)) {

        set_visibility(temp_node->unique_node_id(), in_camera_uuid, true);

        if (temp_node->get_parent() != nullptr)
        {
            temp_node = temp_node->get_parent();
        }

    }

}

////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshRenderer::pull_up_visibility(
    gua::node::Node* current_node,
    int64_t current_frame_id,
    std::size_t in_camera_uuid)
{

    // store the node pointer
    auto temp_node = current_node;
    //std::cout<< "current node name " << temp_node->get_name()<<std::endl;
    // pull up algorithm as stated by CHC paper
    while(!get_visibility(temp_node->unique_node_id(), in_camera_uuid)) {

        set_visibility(temp_node->unique_node_id(), in_camera_uuid, true);
        set_last_visibility_checked_result(temp_node->unique_node_id(), in_camera_uuid, current_frame_id, true);

        if (temp_node->get_parent() != nullptr)
        {
            temp_node = temp_node->get_parent();
        }

    }

}
////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshRenderer::render_visible_leaf(
    gua::node::Node* current_query_node,
    RenderContext const& ctx,
    Pipeline& pipe,
    RenderTarget& render_target,
    MaterialShader* current_material,
    std::shared_ptr<ShaderProgram> current_shader,
    scm::gl::rasterizer_state_ptr current_rasterizer_state,
    bool& depth_complexity_vis)
{

    //Resetting states for drawing for leaf nodes
    auto const& glapi = ctx.render_context->opengl_api();
    glapi.glColorMask(true, true, true, true);
    ctx.render_context->set_depth_stencil_state(default_depth_test_);
    ctx.render_context->set_blend_state(default_blend_state_);
    ctx.render_context->apply();


    //make sure that we currently have a trimesh node in our hands
    if(std::type_index(typeid(node::TriMeshNode)) == std::type_index(typeid(*current_query_node)) ) {

        //std::cout << "ASSUMING THAT " << current_node->get_name() << " is a trimeshnode" << std::endl;
        auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(current_query_node));

        if(!depth_complexity_vis) {
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

////////////////////////////////////////////////////////////////////////////////
void OcclusionCullingTriMeshRenderer::unbind_and_reset(RenderContext const& ctx, RenderTarget& render_target) {
    render_target.unbind(ctx);

    auto const& glapi = ctx.render_context->opengl_api();
    glapi.glColorMask(true, true, true, true);
    ctx.render_context->set_depth_stencil_state(default_depth_test_);
    ctx.render_context->set_blend_state(default_blend_state_);
    ctx.render_context->apply();

    ctx.render_context->reset_state_objects();
    ctx.render_context->sync();
}

////////////////////////////////////////////////////////////////////////////////


bool OcclusionCullingTriMeshRenderer::check_children_surface_area(std::vector<gua::node::Node*> const& in_parent_nodes) const {

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


    float const smax = 1.4f;
    bool is_tighter = (children_surface_area )  <= (parent_surface_area * smax);

    return is_tighter;

}

void OcclusionCullingTriMeshRenderer::instanced_array_draw(
    std::vector<gua::node::Node*> const& leaf_node_vector,
    RenderContext const& ctx,
    std::shared_ptr<ShaderProgram>& current_shader,
    size_t in_camera_uuid,
    size_t current_frame_id)
{
    uint32_t current_instance_ID = 0;


    for (auto const& leaf_node: leaf_node_vector)
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

    glapi.glDrawArraysInstanced(GL_TRIANGLES, 0, 36, current_instance_ID);

}

bool OcclusionCullingTriMeshRenderer::front_to_back_raycast(gua::node::Node* lhs, gua::node::Node* rhs) {
    return true;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
