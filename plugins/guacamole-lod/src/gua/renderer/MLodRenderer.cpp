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
#include <gua/renderer/MLodRenderer.hpp>

// guacamole headers
#include <gua/renderer/LodResource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/MLodNode.hpp>
#include <gua/renderer/View.hpp>

#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>
#include <scm/gl_core/shader_objects.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>
#include <lamure/ren/camera.h>
#include <lamure/ren/policy.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/controller.h>
#include <boost/assign/list_of.hpp>
#include <gua/renderer/MLodPass.hpp>

namespace
{
gua::math::vec2ui get_handle(scm::gl::texture_image_ptr const& tex)
{
    uint64_t handle = tex->native_handle();
    return gua::math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

} // namespace

namespace gua
{
//////////////////////////////////////////////////////////////////////////////
MLodRenderer::MLodRenderer(RenderContext const& ctx, SubstitutionMap const& smap)
    :
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
      VTRenderer(ctx, smap),
#endif
      gpu_resources_created_(false), current_rendertarget_width_(0), current_rendertarget_height_(0)
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("resources/shaders/mlod/tri_mesh_lod_shader.vert");
    std::string f_shader = factory.read_shader_file("resources/shaders/mlod/tri_mesh_lod_shader.frag");
#else
    std::string v_shader = Resources::lookup_shader("shaders/mlod/tri_mesh_lod_shader.vert");
    std::string f_shader = Resources::lookup_shader("shaders/mlod/tri_mesh_lod_shader.frag");
#endif

    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, v_shader));
    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

///////////////////////////////////////////////////////////////////////////////
void MLodRenderer::create_state_objects(RenderContext const& ctx)
{
    rs_cull_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
    rs_cull_none_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ShaderProgram>
MLodRenderer::_get_material_program(MaterialShader* material, std::shared_ptr<ShaderProgram> const& current_program, bool& program_changed, gua::node::MLodNode* mlod_node)
{
    auto shader_iterator = programs_.find(material);
    if(shader_iterator == programs_.end())
    {
        try
        {
            _initialize_tri_mesh_lod_program(material, mlod_node);
            program_changed = true;
            return programs_.at(material);
        }
        catch(std::exception& e)
        {
            Logger::LOG_WARNING << "LodPass::_get_material_program(): Cannot create material for tri_mesh_lod program: " << e.what() << std::endl;
            return std::shared_ptr<ShaderProgram>();
        }
    }
    else
    {
        if(current_program == shader_iterator->second)
        {
            program_changed = false;
            return current_program;
        }
        else
        {
            program_changed = true;
            return shader_iterator->second;
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
void MLodRenderer::_initialize_tri_mesh_lod_program(MaterialShader* material, gua::node::MLodNode* mlod_node)
{
    if(!programs_.count(material))
    {
        auto program = std::make_shared<ShaderProgram>();

        auto smap = global_substitution_map_;
        for(const auto& i : material->generate_substitution_map())
        {
            smap[i.first] = i.second;
        }

#ifndef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
        program->set_shaders(program_stages_, std::list<std::string>(), false, smap);
#else
        bool virtual_texturing_enabled = mlod_node->get_material()->get_enable_virtual_texturing();
        program->set_shaders(program_stages_, std::list<std::string>(), false, smap, virtual_texturing_enabled);
#endif
        programs_[material] = program;
    }
    assert(programs_.count(material));
}

///////////////////////////////////////////////////////////////////////////////
void MLodRenderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc)
{
    RenderContext const& ctx(pipe.get_context());

    ///////////////////////////////////////////////////////////////////////////
    //  retrieve current view state
    ///////////////////////////////////////////////////////////////////////////
    auto& scene = *pipe.current_viewstate().scene;
    auto const& camera = pipe.current_viewstate().camera;
    auto const& frustum = pipe.current_viewstate().frustum;
    auto& target = *pipe.current_viewstate().target;

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string cpu_query_name_plod_total = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / LodPass";
    pipe.begin_cpu_query(cpu_query_name_plod_total);
#endif

    ///////////////////////////////////////////////////////////////////////////
    //  obtain nodes
    ///////////////////////////////////////////////////////////////////////////
    auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::MLodNode))));

    if(sorted_objects == scene.nodes.end() || sorted_objects->second.empty())
    {
        return; // return if no nodes in scene
    }

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
        return reinterpret_cast<node::MLodNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::MLodNode*>(b)->get_material()->get_shader();
    });

    ///////////////////////////////////////////////////////////////////////////
    // resource initialization
    ///////////////////////////////////////////////////////////////////////////
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

    // allocate GPU resources if necessary
    bool resize_resource_containers = false;
    bool render_resolution_changed = current_rendertarget_width_ != render_target_dims[0] || current_rendertarget_height_ != render_target_dims[1];

    if(!gpu_resources_created_ || render_resolution_changed)
    {
        current_rendertarget_width_ = render_target_dims[0];
        current_rendertarget_height_ = render_target_dims[1];
        //_create_gpu_resources(ctx, render_target_dims, resize_resource_containers);
        gpu_resources_created_ = true;
    }

    ///////////////////////////////////////////////////////////////////////////
    // prepare PBR-cut update
    ///////////////////////////////////////////////////////////////////////////
    lamure::context_t context_id = _lamure_register_context(ctx);

    // TODO: can we use  pipe.get_scene().culling_frustum here?
    std::vector<math::vec3> frustum_corner_values = frustum.get_corners();
    float top_minus_bottom = scm::math::length((frustum_corner_values[2]) - (frustum_corner_values[0]));

    float height_divided_by_top_minus_bottom = render_target_dims[1] / (top_minus_bottom);

    // create lamure camera out of gua camera values
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();

    std::size_t pipe_id = (size_t)&pipe;
    std::size_t camera_id = pipe.current_viewstate().viewpoint_uuid;
    unsigned char view_direction = (unsigned char)pipe.current_viewstate().view_direction;

    std::size_t gua_view_id = (camera_id << 8) | (std::size_t(view_direction));

    lamure::view_t lamure_view_id = controller->deduce_view_id(context_id, gua_view_id);

    int view_id(camera.config.get_view_id());

    lamure::ren::camera cut_update_cam(lamure_view_id, frustum.get_clip_near(), math::mat4f(frustum.get_view()), math::mat4f(frustum.get_projection()));

    cuts->send_camera(context_id, lamure_view_id, cut_update_cam);
    cuts->send_height_divided_by_top_minus_bottom(context_id, lamure_view_id, height_divided_by_top_minus_bottom);

    std::unordered_map<node::MLodNode*, lamure::ren::cut*> cut_map;
    std::unordered_map<lamure::model_t, std::unordered_set<lamure::node_t>> nodes_in_frustum_per_model;

    bool write_depth = true;
    target.bind(ctx, write_depth);
    target.set_viewport(ctx);

    // loop through all models and perform frustum culling
    for(auto const& object : sorted_objects->second)
    {
        auto mlod_node(reinterpret_cast<node::MLodNode*>(object));

        lamure::model_t model_id = controller->deduce_model_id(mlod_node->get_geometry_description());

        auto const& scm_model_matrix = mlod_node->get_cached_world_transform();
        //auto const& scm_model_matrix = mlod_node->get_world_transform();
        
        //auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
        //auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
        // auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

        cuts->send_transform(context_id, model_id, math::mat4f(scm_model_matrix));
        cuts->send_rendered(context_id, model_id);
        cuts->send_threshold(context_id, model_id, mlod_node->get_error_threshold());

        // update current model matrix for LodLibrary in order to make bundle pick work
        database->get_model(model_id)->set_transform(math::mat4f(scm_model_matrix));

        lamure::ren::cut& cut = cuts->get_cut(context_id, lamure_view_id, model_id);
        cut_map.insert(std::make_pair(mlod_node, &cut));

        std::vector<lamure::ren::cut::node_slot_aggregate>& node_list = cut.complete_set();

        // perform frustum culling
        lamure::ren::bvh * bvh = database->get_model(model_id)->get_bvh();
        bvh->set_min_lod_depth(mlod_node->get_min_lod_depth());

        scm::gl::frustum const& culling_frustum = cut_update_cam.get_frustum_by_model(math::mat4f(scm_model_matrix));
        //auto culling_frustum = scm::gl::frustum(scm::math::mat4f(scm_model_view_projection_matrix));

        std::vector<scm::gl::boxf> const& model_bounding_boxes = bvh->get_bounding_boxes();

        std::unordered_set<lamure::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[model_id];

        auto global_clipping_planes = scene.clipping_planes;
        unsigned num_global_clipping_planes = global_clipping_planes.size();
        auto scm_transpose_model_matrix = scm::math::transpose(scm_model_matrix);
        auto scm_inverse_model_matrix = scm::math::inverse(scm_model_matrix);

        for(unsigned plane_idx = 0; plane_idx < num_global_clipping_planes; ++plane_idx)
        {
            scm::math::vec4d plane_vec = scm::math::vec4d(global_clipping_planes[plane_idx]);

            scm::math::vec3d xyz_comp = scm::math::vec3d(plane_vec);

            double d = -plane_vec.w;

            scm::math::vec4d O = scm::math::vec4d(xyz_comp * d, 1.0);
            scm::math::vec4d N = scm::math::vec4d(xyz_comp, 0.0);
            O = scm_inverse_model_matrix * O;
            N = scm_transpose_model_matrix * N;
            xyz_comp = scm::math::vec3d(N);
            d = scm::math::dot(scm::math::vec3d(O), scm::math::vec3d(N));

            global_clipping_planes[plane_idx] = scm::math::vec4d(xyz_comp, -d);
        }

        for(auto const& n : node_list)
        {
            if(culling_frustum.classify(model_bounding_boxes[n.node_id_]) != 1)
            {
                if(num_global_clipping_planes == 0 || _intersects(model_bounding_boxes[n.node_id_], global_clipping_planes))
                {
                    nodes_in_frustum.insert(n.node_id_);
                }
            }
        }
    }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / MLodRenderer";
    pipe.begin_gpu_query(ctx, gpu_query_name);
#endif

    MaterialShader* current_material(nullptr);
    std::shared_ptr<ShaderProgram> current_material_program;

    bool program_changed = false;

    auto current_rasterizer_state = rs_cull_back_;

    for(auto const& object : sorted_objects->second)
    {
        auto mlod_node(reinterpret_cast<node::MLodNode*>(object));

        lamure::model_t model_id = controller->deduce_model_id(mlod_node->get_geometry_description());

        current_material = mlod_node->get_material()->get_shader();

        current_material_program = _get_material_program(current_material, current_material_program, program_changed, mlod_node);

        if(current_material_program)
        {
            current_material_program->use(ctx);
            current_material_program->set_uniform(ctx,
                                                  math::vec2i(target.get_width(), target.get_height()),
                                                  "gua_resolution"); // TODO: pass gua_resolution. Probably should be somehow else implemented
            current_material_program->set_uniform(ctx, 1.0f / target.get_width(), "gua_texel_width");
            current_material_program->set_uniform(ctx, 1.0f / target.get_height(), "gua_texel_height");
            // hack
            current_material_program->set_uniform(ctx, ::get_handle(target.get_depth_buffer()), "gua_gbuffer_depth");

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
            if(!pipe.current_viewstate().shadow_mode)
            {
                VTContextState* vt_state = &VTBackend::get_instance().get_state(pipe.current_viewstate().camera.uuid);

                if(vt_state && vt_state->has_camera_)
                {
                    current_material_program->set_uniform(ctx, vt_state->feedback_enabled_, "enable_feedback");
                }
            }
#endif
        }

        auto const& mlod_resource = mlod_node->get_geometry();

        std::unordered_set<lamure::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[model_id];

        if(mlod_resource && current_material_program)
        {
            if(program_changed)
            {
                current_material_program->unuse(ctx);
                current_material_program->use(ctx);
            }

			auto const node_world_transform = mlod_node->get_latest_cached_world_transform(ctx.render_window);

			auto model_view_mat = scene.rendering_frustum.get_view() * node_world_transform;
			UniformValue normal_mat(math::mat4f(scm::math::transpose(scm::math::inverse(node_world_transform))));

			int rendering_mode = pipe.current_viewstate().shadow_mode ? (mlod_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

			current_material_program->apply_uniform(ctx, "gua_model_matrix", math::mat4f(node_world_transform));
			current_material_program->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
			current_material_program->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
			current_material_program->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);
			
#ifdef oldstuff
            auto const& scm_model_matrix = mlod_node->get_cached_world_transform();
            auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
            // auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
            auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

            int rendering_mode = pipe.current_viewstate().shadow_mode ? (mlod_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

            current_material_program->apply_uniform(ctx, "gua_model_matrix", math::mat4f(mlod_node->get_cached_world_transform()));
            current_material_program->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
            current_material_program->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));
            current_material_program->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);
#endif
            // lowfi shadows dont need material input
            if(rendering_mode != 1)
            {
                mlod_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);
            }

            current_rasterizer_state = mlod_node->get_material()->get_show_back_faces() ? rs_cull_none_ : rs_cull_back_;

            if(ctx.render_context->current_rasterizer_state() != current_rasterizer_state)
            {
                ctx.render_context->set_rasterizer_state(current_rasterizer_state);
                ctx.render_context->apply_state_objects();
            }

            ctx.render_context->apply_program();
            ctx.render_context->apply();

            mlod_resource->draw(ctx,
                                context_id,
                                lamure_view_id,
                                model_id,
                                controller->get_context_memory(context_id, lamure::ren::bvh::primitive_type::TRIMESH, ctx.render_device),
                                nodes_in_frustum,
                                scm::gl::primitive_topology::PRIMITIVE_TRIANGLE_LIST);

            program_changed = false;
        }
        else
        {
            Logger::LOG_WARNING << "MLodRenderer::render(): Cannot find ressources for node: " << mlod_node->get_name() << std::endl;
        }
    }

    current_material_program->unuse(ctx);

    //////////////////////////////////////////////////////////////////////////
    // Draw finished -> unbind g-buffer
    //////////////////////////////////////////////////////////////////////////
    target.unbind(ctx);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_gpu_query(ctx, gpu_query_name);
    pipe.end_cpu_query(cpu_query_name_plod_total);
#endif

    // dispatch cut updates
    if(previous_frame_count_ != ctx.framecount)
    {
        previous_frame_count_ = ctx.framecount;
        controller->dispatch(controller->deduce_context_id(ctx.id), ctx.render_device);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
bool MLodRenderer::_intersects(scm::gl::boxf const& bbox, std::vector<math::vec4> const& global_planes) const
{
    auto outside = [](math::vec4 const& plane, scm::math::vec3f const& point) { return (plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]) < 0; };

    for(auto const& plane : global_planes)
    {
        auto bbox_max(bbox.max_vertex());
        auto p(bbox.min_vertex());
        if(plane[0] >= 0)
            p[0] = bbox_max[0];
        if(plane[1] >= 0)
            p[1] = bbox_max[1];
        if(plane[2] >= 0)
            p[2] = bbox_max[2];

        // is the positive vertex outside?
        if(outside(plane, p))
        {
            return false;
        }
    }

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::vector<math::vec3> MLodRenderer::_get_frustum_corners_vs(gua::Frustum const& frustum) const
{
    std::vector<math::vec4> tmp(8);
    std::vector<math::vec3> result(8);

    auto inverse_transform(scm::math::inverse(frustum.get_projection()));

    tmp[0] = inverse_transform * math::vec4(-1, -1, -1, 1);
    tmp[1] = inverse_transform * math::vec4(-1, -1, 1, 1);
    tmp[2] = inverse_transform * math::vec4(-1, 1, -1, 1);
    tmp[3] = inverse_transform * math::vec4(-1, 1, 1, 1);
    tmp[4] = inverse_transform * math::vec4(1, -1, -1, 1);
    tmp[5] = inverse_transform * math::vec4(1, -1, 1, 1);
    tmp[6] = inverse_transform * math::vec4(1, 1, -1, 1);
    tmp[7] = inverse_transform * math::vec4(1, 1, 1, 1);

    for(int i(0); i < 8; ++i)
    {
        result[i] = tmp[i] / tmp[i][3];
    }

    return result;
}

/////////////////////////////////////////////////////////////////////////////////////////////
lamure::context_t MLodRenderer::_lamure_register_context(gua::RenderContext const& ctx)
{
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    if(previous_frame_count_ != ctx.framecount)
    {
        controller->reset_system();
    }
    return controller->deduce_context_id(ctx.id);
}

} // namespace gua
