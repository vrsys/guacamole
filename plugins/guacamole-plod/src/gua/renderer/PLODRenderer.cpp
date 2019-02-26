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
#include <gua/renderer/PLODRenderer.hpp>

// guacamole headers
#include <gua/renderer/PLODResource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/PLODNode.hpp>
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
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
#include <pbr/ren/camera.h>
#include <pbr/ren/policy.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/cut_database.h>
#include <pbr/ren/controller.h>
#include <pbr/ren/raw_point_cloud.h>
#include <boost/assign/list_of.hpp>

namespace gua
{
bool PLODRenderer::_intersects(scm::gl::boxf const& bbox, std::vector<math::vec4f> const& global_planes) const
{
    auto outside = [](math::vec4f const& plane, scm::math::vec3f const& point) { return (plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3]) < 0; };

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

std::vector<math::vec3> PLODRenderer::_get_frustum_corners_vs(gua::Frustum const& frustum) const
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

//////////////////////////////////////////////////////////////////////////////
PLODRenderer::PLODRenderer()
    : shaders_loaded_(false), gpu_resources_already_created_(false), depth_pass_program_(nullptr), normalization_pass_program_(nullptr), current_rendertarget_width_(0), current_rendertarget_height_(0)
{
    _load_shaders();
}

//////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;

        log_to_lin_conversion_shader_stages_.clear();
        log_to_lin_conversion_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p00_log_to_lin_conversion.vert")));
        log_to_lin_conversion_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p00_log_to_lin_conversion.frag")));

        depth_pass_shader_stages_.clear();
        depth_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_depth.vert")));
        depth_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_depth.geom")));
        depth_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_depth.frag")));

        accumulation_pass_shader_stages_.clear();
        accumulation_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p02_accumulation.vert")));
        accumulation_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p02_accumulation.geom")));
        accumulation_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p02_accumulation.frag")));

        normalization_pass_shader_stages_.clear();
        normalization_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p03_normalization.vert")));
        normalization_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p03_normalization.frag")));

        shadow_pass_shader_stages_.clear();
        shadow_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_shadow.vert")));
        shadow_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_shadow.geom")));
        shadow_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/gbuffer/plod/p01_shadow.frag")));

        shaders_loaded_ = true;
    }
}

//////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_initialize_log_to_lin_conversion_pass_program()
{
    if(!log_to_lin_conversion_pass_program_)
    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(log_to_lin_conversion_shader_stages_);
        log_to_lin_conversion_pass_program_ = new_program;
    }
    assert(log_to_lin_conversion_pass_program_);
}

//////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_initialize_depth_pass_program()
{
    if(!depth_pass_program_)
    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(depth_pass_shader_stages_);
        depth_pass_program_ = new_program;
    }
    assert(depth_pass_program_);
}

//////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_initialize_accumulation_pass_program(MaterialShader* material)
{
    if(!accumulation_pass_programs_.count(material))
    {
        auto program = std::make_shared<ShaderProgram>();

        auto smap = global_substitution_map_;
        for(const auto& i : material->generate_substitution_map())
        {
            smap[i.first] = i.second;
        }

        program->set_shaders(accumulation_pass_shader_stages_, std::list<std::string>(), false, smap);
        accumulation_pass_programs_[material] = program;
    }
    assert(accumulation_pass_programs_.count(material));
}

//////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_initialize_normalization_pass_program()
{
    if(!normalization_pass_program_)
    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(normalization_pass_shader_stages_);
        normalization_pass_program_ = new_program;
    }
    assert(normalization_pass_program_);
}

//////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_initialize_shadow_pass_program()
{
    if(!shadow_pass_program_)
    {
        auto new_program = std::make_shared<ShaderProgram>();
        new_program->set_shaders(shadow_pass_shader_stages_);
        shadow_pass_program_ = new_program;
    }
    assert(shadow_pass_program_);
}

///////////////////////////////////////////////////////////////////////////////
void PLODRenderer::_create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, bool resize_resource_containers)
{
    /*depth_pass_log_depth_result_ = ctx.render_device
      ->create_texture_2d(render_target_dims,
                          scm::gl::FORMAT_R_32F,
                          1, 1, 1);
    */

    depth_pass_linear_depth_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_D32F, 1, 1, 1);

    accumulation_pass_color_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGB_16F, 1, 1, 1);

    accumulation_pass_normal_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGB_16F, 1, 1, 1);

    accumulation_pass_pbr_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RGB_16F, 1, 1, 1);

    accumulation_pass_weight_and_depth_result_ = ctx.render_device->create_texture_2d(render_target_dims, scm::gl::FORMAT_RG_32F, 1, 1, 1);

    log_to_lin_gua_depth_conversion_pass_fbo_ = ctx.render_device->create_frame_buffer();
    /*log_to_lin_gua_depth_conversion_pass_fbo_->attach_color_buffer(0,
                                                                   depth_pass_linear_depth_result_);*/
    log_to_lin_gua_depth_conversion_pass_fbo_->attach_depth_stencil_buffer(depth_pass_linear_depth_result_);

    depth_pass_result_fbo_ = ctx.render_device->create_frame_buffer();
    depth_pass_result_fbo_->clear_attachments();
    depth_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_linear_depth_result_);
    /*depth_pass_result_fbo_->attach_color_buffer(0,
                                                depth_pass_log_depth_result_);
    */

    accumulation_pass_result_fbo_ = ctx.render_device->create_frame_buffer();
    accumulation_pass_result_fbo_->clear_attachments();
    accumulation_pass_result_fbo_->attach_color_buffer(0, accumulation_pass_color_result_);
    accumulation_pass_result_fbo_->attach_color_buffer(1, accumulation_pass_normal_result_);
    accumulation_pass_result_fbo_->attach_color_buffer(2, accumulation_pass_pbr_result_);
    accumulation_pass_result_fbo_->attach_color_buffer(3, accumulation_pass_weight_and_depth_result_);
    // accumulation_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_log_depth_result_);
    accumulation_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_linear_depth_result_);

    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    no_depth_test_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_ALWAYS);

    depth_test_without_writing_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS_EQUAL);

    no_depth_test_with_writing_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_ALWAYS);

    color_accumulation_state_ = ctx.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD);

    no_backface_culling_rasterizer_state_ =
        ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(false));

    fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f)));

    // invalidation before first write
    previous_frame_count_ = UINT_MAX;
}

/////////////////////////////////////////////////////////////////////////////////////////////
pbr::context_t PLODRenderer::_register_context_in_cut_update(gua::RenderContext const& ctx)
{
    pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
    if(previous_frame_count_ != ctx.framecount)
    {
        controller->ResetSystem();
    }
    return controller->DeduceContextId(ctx.id);
}

/////////////////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<ShaderProgram> PLODRenderer::_get_material_program(MaterialShader* material, std::shared_ptr<ShaderProgram> const& current_program, bool& program_changed)
{
    auto shader_iterator = accumulation_pass_programs_.find(material);
    if(shader_iterator == accumulation_pass_programs_.end())
    {
        try
        {
            _initialize_accumulation_pass_program(material);
            program_changed = true;
            return accumulation_pass_programs_.at(material);
        }
        catch(std::exception& e)
        {
            Logger::LOG_WARNING << "PLODPass::_get_material_program(): Cannot create material for accumulation pass program: " << e.what() << std::endl;
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

///////////////////////////////////////////////////////////////////////////////
void PLODRenderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc)
{
    RenderContext const& ctx(pipe.get_context());

    ///////////////////////////////////////////////////////////////////////////
    //  retrieve current view state
    ///////////////////////////////////////////////////////////////////////////
    auto& scene = *pipe.current_viewstate().scene;
    auto const& camera = pipe.current_viewstate().camera;
    auto const& frustum = pipe.current_viewstate().frustum;
    auto& target = *pipe.current_viewstate().target;

    std::string cpu_query_name_plod_total = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLODPass";
    pipe.begin_cpu_query(cpu_query_name_plod_total);

    ///////////////////////////////////////////////////////////////////////////
    //  sort nodes
    ///////////////////////////////////////////////////////////////////////////
    auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::PLODNode))));

    if(sorted_objects == scene.nodes.end() || sorted_objects->second.empty())
    {
        return; // return if no nodes in scene
    }

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
        return reinterpret_cast<node::PLODNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::PLODNode*>(b)->get_material()->get_shader();
    });

    ///////////////////////////////////////////////////////////////////////////
    // resource initialization
    ///////////////////////////////////////////////////////////////////////////
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

    // allocate GPU resources if necessary
    bool resize_resource_containers = false;
    bool render_resolution_changed = current_rendertarget_width_ != render_target_dims[0] || current_rendertarget_height_ != render_target_dims[1];

    if(!gpu_resources_already_created_ || render_resolution_changed)
    {
        current_rendertarget_width_ = render_target_dims[0];
        current_rendertarget_height_ = render_target_dims[1];
        _create_gpu_resources(ctx, render_target_dims, resize_resource_containers);
        gpu_resources_already_created_ = true;
    }

    ctx.render_context->clear_color_buffer(depth_pass_result_fbo_, 0, scm::math::vec4(1.0f, 1.0f, 1.0f, 1.0f));

    ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_, 0, scm::math::vec3f(0.0f, 0.0f, 0.0f));

    ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_, 1, scm::math::vec3f(0.0f, 0.0f, 0.0f));

    ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_, 2, scm::math::vec3f(0.0f, 0.0f, 0.0f));

    ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_, 3, scm::math::vec2f(0.0f, 0.0f));

    ///////////////////////////////////////////////////////////////////////////
    // program initialization
    ///////////////////////////////////////////////////////////////////////////
    try
    {
        if(!log_to_lin_conversion_pass_program_)
        {
            _initialize_log_to_lin_conversion_pass_program();
        }

        if(!depth_pass_program_)
        {
            _initialize_depth_pass_program();
        }

        if(!normalization_pass_program_)
        {
            _initialize_normalization_pass_program();
        }

        if(!shadow_pass_program_)
        {
            _initialize_shadow_pass_program();
        }

        assert(log_to_lin_conversion_pass_program_ && depth_pass_program_ && normalization_pass_program_);
    }
    catch(std::exception& e)
    {
        gua::Logger::LOG_ERROR << "Error: PLODRenderer::render() : Failed to create programs. " << e.what() << std::endl;
    }

    target.set_viewport(ctx);

    ///////////////////////////////////////////////////////////////////////////
    // prepare PBR-cut update
    ///////////////////////////////////////////////////////////////////////////
    pbr::context_t context_id = _register_context_in_cut_update(ctx);

    // TODO: can we use  pipe.get_scene().culling_frustum here?
    std::vector<math::vec3> frustum_corner_values = frustum.get_corners();
    float top_minus_bottom = scm::math::length((frustum_corner_values[2]) - (frustum_corner_values[0]));

    float height_divided_by_top_minus_bottom = render_target_dims[1] / (top_minus_bottom);

    // create pbr camera out of gua camera values
    pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

    std::size_t pipe_id = (size_t)&pipe;
    std::size_t camera_id = pipe.current_viewstate().viewpoint_uuid;
    unsigned char view_direction = (unsigned char)pipe.current_viewstate().view_direction;

    std::size_t gua_view_id = (camera_id << 8) | (std::size_t(view_direction));

    // std::cout << " View UUID : " /*<< std::setfill('0') << std::setw(32) << gua_view_id */<<
    //" Near clip : " << frustum.get_clip_near() <<
    //" Far clip : " << frustum.get_clip_far() <<
    //" Resolution : " << render_target_dims <<
    //" Camera_id : " << camera_id <<
    //" V Direction : " << (int)view_direction << std::endl;

    pbr::view_t pbr_view_id = controller->DeduceViewId(context_id, gua_view_id);

    pbr::ren::Camera cut_update_cam(pbr_view_id, frustum.get_clip_near(), math::mat4f(frustum.get_view()), math::mat4f(frustum.get_projection()));

    cuts->SendCamera(context_id, pbr_view_id, cut_update_cam);
    cuts->SendHeightDividedByTopMinusBottom(context_id, pbr_view_id, height_divided_by_top_minus_bottom);

    auto& gua_depth_buffer = target.get_depth_buffer();

    std::unordered_map<node::PLODNode*, pbr::ren::Cut*> cut_map;
    std::unordered_map<pbr::model_t, std::unordered_set<pbr::node_t>> nodes_in_frustum_per_model;

    // loop through all models and perform frustum culling
    for(auto const& object : sorted_objects->second)
    {
        auto plod_node(reinterpret_cast<node::PLODNode*>(object));

        pbr::model_t model_id = controller->DeduceModelId(plod_node->get_geometry_description());

        auto const& scm_model_matrix = plod_node->get_cached_world_transform();
        auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
        auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
        auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

        cuts->SendTransform(context_id, model_id, math::mat4f(scm_model_matrix));
        cuts->SendRendered(context_id, model_id);
        cuts->SendThreshold(context_id, model_id, plod_node->get_error_threshold());

        // update current model matrix for PLODLibrary in order to make bundle pick work
        database->GetModel(model_id)->set_transform(math::mat4f(scm_model_matrix));

        pbr::ren::Cut& cut = cuts->GetCut(context_id, pbr_view_id, model_id);
        cut_map.insert(std::make_pair(plod_node, &cut));

        std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();
        // std::cout << " # Rendered Nodes : " << node_list.size() << std::endl;

        // perform frustum culling
        pbr::ren::Bvh const* bvh = database->GetModel(model_id)->bvh();
        scm::gl::frustum const& culling_frustum = cut_update_cam.GetFrustumByModel(math::mat4f(scm_model_matrix));

        std::vector<scm::gl::boxf> const& model_bounding_boxes = bvh->bounding_boxes();

        std::unordered_set<pbr::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[model_id];

        // auto global_clipping_planes = scene.clipping_planes;
        std::vector<math::vec4f> global_clipping_planes{};
        for(auto const& plane : scene.clipping_planes)
        {
            global_clipping_planes.push_back(math::vec4f(plane));
        }
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

    if(!pipe.current_viewstate().shadow_mode)
    { // normal rendering branch
        ///////////////////////////////////////////////////////////////////////////
        // fullscreen gbuffer depth_conversion_pass
        ///////////////////////////////////////////////////////////////////////////
        {
            scm::gl::context_all_guard context_guard(ctx.render_context);

            if(!log_to_lin_conversion_pass_program_)
            {
                std::cout << "Log to lin pass program not instanciated\n";
            }

            std::string const gpu_query_name_depth_conversion = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLODRenderer::DepthConversionPass";
            pipe.begin_gpu_query(ctx, gpu_query_name_depth_conversion);

            log_to_lin_conversion_pass_program_->use(ctx);

            ctx.render_context->set_depth_stencil_state(no_depth_test_with_writing_depth_stencil_state_);

            ctx.render_context->set_frame_buffer(log_to_lin_gua_depth_conversion_pass_fbo_);

            ctx.render_context->clear_depth_stencil_buffer(log_to_lin_gua_depth_conversion_pass_fbo_);

            ctx.render_context->bind_texture(gua_depth_buffer, nearest_sampler_state_, 0);
            log_to_lin_conversion_pass_program_->apply_uniform(ctx, "gua_log_depth_buffer", 0);

            float width = render_target_dims[0];
            float height = render_target_dims[1];
            log_to_lin_conversion_pass_program_->apply_uniform(ctx, "win_width", (width));

            log_to_lin_conversion_pass_program_->apply_uniform(ctx, "win_height", (height));

            ctx.render_context->apply();

            fullscreen_quad_->draw(ctx.render_context);

            // pipe.end_cpu_query("CPU : PLODRenderer::depth_conversion_pass");
            // pipe.end_gpu_query(ctx, "GPU : PLODRenderer::depth_conversion_pass");

            log_to_lin_conversion_pass_program_->unuse(ctx);

            pipe.end_gpu_query(ctx, gpu_query_name_depth_conversion);
        }

        //////////////////////////////////////////////////////////////////////////
        // 1. depth pass
        //////////////////////////////////////////////////////////////////////////

        {
            scm::gl::context_all_guard context_guard(ctx.render_context);

            std::string const gpu_query_name_depth_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLODRenderer::DepthPass";
            pipe.begin_gpu_query(ctx, gpu_query_name_depth_pass);

            ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);
            depth_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_linear_depth_result_);
            ctx.render_context->set_frame_buffer(depth_pass_result_fbo_);

            depth_pass_program_->use(ctx);

            // loop through all models and render depth pass
            for(auto const& object : sorted_objects->second)
            {
                auto plod_node(reinterpret_cast<node::PLODNode*>(object));

                pbr::model_t model_id = controller->DeduceModelId(plod_node->get_geometry_description());

                auto const& scm_model_matrix = plod_node->get_cached_world_transform();
                auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
                auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
                auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

                depth_pass_program_->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
                depth_pass_program_->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
                depth_pass_program_->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
                depth_pass_program_->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));

                depth_pass_program_->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
                depth_pass_program_->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());

                ctx.render_context->apply();

                std::unordered_set<pbr::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[model_id];

                auto const& plod_resource = plod_node->get_geometry();

                if(plod_resource && depth_pass_program_)
                {
                    plod_resource->draw(ctx, context_id, pbr_view_id, model_id, controller->GetContextMemory(context_id, ctx.render_device), nodes_in_frustum);
                }
                else
                {
                    Logger::LOG_WARNING << "PLODRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
                }
            }

            depth_pass_program_->unuse(ctx);

            pipe.end_gpu_query(ctx, gpu_query_name_depth_pass);
        }

        //////////////////////////////////////////////////////////////////////////
        // 2. accumulation pass
        //////////////////////////////////////////////////////////////////////////
        MaterialShader* current_material(nullptr);
        std::shared_ptr<ShaderProgram> current_material_program;

        {
            scm::gl::context_all_guard context_guard(ctx.render_context);

            std::string const gpu_query_name_accum_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLODRenderer::AccumulationPass";
            pipe.begin_gpu_query(ctx, gpu_query_name_accum_pass);

            ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);
            ctx.render_context->set_depth_stencil_state(depth_test_without_writing_depth_stencil_state_);
            ctx.render_context->set_blend_state(color_accumulation_state_);
            ctx.render_context->set_frame_buffer(accumulation_pass_result_fbo_);

            accumulation_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_linear_depth_result_);

            int view_id(camera.config.get_view_id());

            bool program_changed = false;
            // loop through all models and render accumulation pass
            for(auto const& object : sorted_objects->second)
            {
                auto plod_node(reinterpret_cast<node::PLODNode*>(object));
                pbr::model_t model_id = controller->DeduceModelId(plod_node->get_geometry_description());

                auto& cut = *(cut_map.at(plod_node));
                std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();

                current_material = plod_node->get_material()->get_shader();

                current_material_program = _get_material_program(current_material, current_material_program, program_changed);

                ctx.render_context->bind_texture(depth_pass_linear_depth_result_, nearest_sampler_state_, 0);
                current_material_program->apply_uniform(ctx, "p01_linear_depth_texture", 0);

                ctx.render_context->apply();

                auto plod_resource = plod_node->get_geometry();

                // retrieve frustum culling results
                std::unordered_set<pbr::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[model_id];

                if(plod_resource && current_material_program)
                {
                    if(program_changed)
                    {
                        current_material_program->unuse(ctx);
                        current_material_program->use(ctx);
                    }

                    auto const& scm_model_matrix = plod_node->get_cached_world_transform();
                    auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
                    auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
                    auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

                    current_material_program->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
                    current_material_program->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
                    current_material_program->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
                    current_material_program->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));
                    current_material_program->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
                    current_material_program->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());

                    plod_node->get_material()->apply_uniforms(ctx, current_material_program.get(), view_id);

                    plod_resource->draw(ctx, context_id, pbr_view_id, model_id, controller->GetContextMemory(context_id, ctx.render_device), nodes_in_frustum);

                    program_changed = false;
                }
                else
                {
                    Logger::LOG_WARNING << "PLODRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
                }
            }

            current_material_program->unuse(ctx);

            pipe.end_gpu_query(ctx, gpu_query_name_accum_pass);
        }

        bool write_depth = true;
        target.bind(ctx, write_depth);

        //////////////////////////////////////////////////////////////////////////
        // 3. normalization pass
        //////////////////////////////////////////////////////////////////////////
        {
            scm::gl::context_all_guard context_guard(ctx.render_context);

            std::string const gpu_query_name_normalization_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLODRenderer::NormalizationPass";
            pipe.begin_gpu_query(ctx, gpu_query_name_normalization_pass);

            normalization_pass_program_->use(ctx);
            {
                // ctx.render_context->set_depth_stencil_state(no_depth_test_with_writing_depth_stencil_state_);

                ctx.render_context->bind_texture(accumulation_pass_color_result_, nearest_sampler_state_, 0);
                normalization_pass_program_->apply_uniform(ctx, "p02_color_texture", 0);

                ctx.render_context->bind_texture(accumulation_pass_normal_result_, nearest_sampler_state_, 1);
                normalization_pass_program_->apply_uniform(ctx, "p02_normal_texture", 1);

                ctx.render_context->bind_texture(accumulation_pass_pbr_result_, nearest_sampler_state_, 2);
                normalization_pass_program_->apply_uniform(ctx, "p02_pbr_texture", 2);

                ctx.render_context->bind_texture(accumulation_pass_weight_and_depth_result_, nearest_sampler_state_, 3);
                normalization_pass_program_->apply_uniform(ctx, "p02_weight_and_depth_texture", 3);

                // ctx.render_context->bind_texture(depth_pass_log_depth_result_, nearest_sampler_state_, 3);
                // current_material_program->apply_uniform(ctx, "p01_log_depth_texture", 3);

                ctx.render_context->apply();

                fullscreen_quad_->draw(ctx.render_context);
            }
            normalization_pass_program_->unuse(ctx);

            pipe.begin_gpu_query(ctx, gpu_query_name_normalization_pass);
        }
    }
    else
    { // shadow branch
        bool write_depth = true;
        target.bind(ctx, write_depth);

        //////////////////////////////////////////////////////////////////////////
        // only pass in this branch: shadow pass
        //////////////////////////////////////////////////////////////////////////

        {
            scm::gl::context_all_guard context_guard(ctx.render_context);

            std::string const gpu_query_name_depth_pass = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / PLODRenderer::DepthPass";
            pipe.begin_gpu_query(ctx, gpu_query_name_depth_pass);

            ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);

            shadow_pass_program_->use(ctx);

            for(auto const& object : sorted_objects->second)
            {
                auto plod_node(reinterpret_cast<node::PLODNode*>(object));

                pbr::model_t model_id = controller->DeduceModelId(plod_node->get_geometry_description());

                auto const& scm_model_matrix = plod_node->get_cached_world_transform();
                auto scm_model_view_matrix = frustum.get_view() * scm_model_matrix;
                auto scm_model_view_projection_matrix = frustum.get_projection() * scm_model_view_matrix;
                auto scm_normal_matrix = scm::math::transpose(scm::math::inverse(scm_model_matrix));

                shadow_pass_program_->apply_uniform(ctx, "gua_model_matrix", math::mat4f(scm_model_matrix));
                shadow_pass_program_->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(scm_model_view_matrix));
                shadow_pass_program_->apply_uniform(ctx, "gua_model_view_projection_matrix", math::mat4f(scm_model_view_projection_matrix));
                shadow_pass_program_->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(scm_normal_matrix));

                shadow_pass_program_->apply_uniform(ctx, "radius_scaling", plod_node->get_radius_scale());
                shadow_pass_program_->apply_uniform(ctx, "enable_backface_culling", plod_node->get_enable_backface_culling_by_normal());

                ctx.render_context->apply();

                auto const& plod_resource = plod_node->get_geometry();

                std::unordered_set<pbr::node_t>& nodes_in_frustum = nodes_in_frustum_per_model[model_id];

                if(plod_resource && depth_pass_program_)
                {
                    plod_resource->draw(ctx, context_id, pbr_view_id, model_id, controller->GetContextMemory(context_id, ctx.render_device), nodes_in_frustum);
                }
                else
                {
                    Logger::LOG_WARNING << "PLODRenderer::render(): Cannot find ressources for node: " << plod_node->get_name() << std::endl;
                }
            }

            shadow_pass_program_->unuse(ctx);
        }
    }
    //////////////////////////////////////////////////////////////////////////
    // Draw finished -> unbind g-buffer
    //////////////////////////////////////////////////////////////////////////
    target.unbind(ctx);

    pipe.end_cpu_query(cpu_query_name_plod_total);

    // dispatch cut updates
    if(previous_frame_count_ != ctx.framecount)
    {
        previous_frame_count_ = ctx.framecount;
        controller->Dispatch(controller->DeduceContextId(ctx.id), ctx.render_device);
    }
}

} // namespace gua
