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
#include <gua/renderer/PLodRenderer.hpp>
#include <gua/renderer/PLodPass.hpp>

// sub rendering passes
#include <gua/renderer/LowQualitySplattingSubRenderer.hpp>
#include <gua/renderer/NormalizationSubRenderer.hpp>
#include <gua/renderer/AccumSubRenderer.hpp>
#include <gua/renderer/DepthSubRenderer.hpp>
#include <gua/renderer/LogToLinSubRenderer.hpp>

// guacamole headers
#include <gua/renderer/LodResource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/PLodNode.hpp>
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
#include <lamure/ren/camera.h>
#include <lamure/ren/policy.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/controller.h>
#include <boost/assign/list_of.hpp>

namespace gua
{
bool PLodRenderer::_intersects(scm::gl::boxf const& bbox, std::vector<math::vec4> const& global_planes) const
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

std::vector<math::vec3> PLodRenderer::_get_frustum_corners_vs(gua::Frustum const& frustum) const
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
PLodRenderer::PLodRenderer()
{
    std::shared_ptr<std::vector<std::shared_ptr<PLodSubRenderer>>> HQ_two_pass_splatting_pipeline_ptr = std::make_shared<std::vector<std::shared_ptr<PLodSubRenderer>>>();
    std::shared_ptr<std::vector<std::shared_ptr<PLodSubRenderer>>> LQ_one_pass_splatting_pipeline_ptr = std::make_shared<std::vector<std::shared_ptr<PLodSubRenderer>>>();

    HQ_two_pass_splatting_pipeline_ptr->push_back(std::make_shared<LogToLinSubRenderer>());
    HQ_two_pass_splatting_pipeline_ptr->push_back(std::make_shared<DepthSubRenderer>());
    HQ_two_pass_splatting_pipeline_ptr->push_back(std::make_shared<AccumSubRenderer>());
    HQ_two_pass_splatting_pipeline_ptr->push_back(std::make_shared<NormalizationSubRenderer>());

    LQ_one_pass_splatting_pipeline_ptr->push_back(std::make_shared<LowQualitySplattingSubRenderer>());

    plod_pipelines_[PLodPassDescription::SurfelRenderMode::HQ_TWO_PASS] = HQ_two_pass_splatting_pipeline_ptr;
    plod_pipelines_[PLodPassDescription::SurfelRenderMode::LQ_ONE_PASS] = LQ_one_pass_splatting_pipeline_ptr;
}

///////////////////////////////////////////////////////////////////////////////
void PLodRenderer::_create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims)
{
    // invalidation before first write
    previous_frame_count_ = UINT_MAX;
}

/////////////////////////////////////////////////////////////////////////////////////////////
void PLodRenderer::_check_for_resource_updates(gua::Pipeline const& pipe, RenderContext const& ctx)
{
    // get current unique view id and resolution
    auto const& camera = pipe.current_viewstate().camera;
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();
    auto camera_id = pipe.current_viewstate().viewpoint_uuid;
    auto view_direction = pipe.current_viewstate().view_direction;
    std::size_t gua_view_id = (camera_id << 8) | (std::size_t(view_direction));

    // check if resources for this view and resolution are already available
    auto view_resources_available = shared_pass_resources_.count(gua_view_id);
    bool resolution_available = false;

    if(view_resources_available)
    {
        auto latest_resolution = shared_pass_resources_[gua_view_id].first;
        resolution_available = (latest_resolution == render_target_dims);
    }
    else
    {
        // if not available create entry
        shared_pass_resources_[gua_view_id] = {render_target_dims, gua::plod_shared_resources()};
    }

    // if not, allocate
    if(!view_resources_available || !resolution_available)
    {
        for(auto const& pipeline_ptrs : plod_pipelines_)
        {
            for(auto const& pass : *(pipeline_ptrs.second))
            {
                auto& resources = shared_pass_resources_.at(gua_view_id).second;
                pass->create_gpu_resources(ctx, render_target_dims, resources);
            }
        }
        _create_gpu_resources(ctx, render_target_dims);
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
lamure::context_t PLodRenderer::_register_context_in_cut_update(gua::RenderContext const& ctx)
{
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    if(previous_frame_count_ != ctx.framecount)
    {
        controller->reset_system();
    }
    return controller->deduce_context_id(ctx.id);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void PLodRenderer::perform_frustum_culling_for_scene(std::vector<node::Node*>& models,
                                                     std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& culling_results_per_model,
                                                     std::unordered_map<node::PLodNode*, lamure::ren::cut*> cut_map,
                                                     lamure::ren::camera const& cut_update_cam,
                                                     gua::Pipeline& pipe) const
{
    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();

    auto& scene = *pipe.current_viewstate().scene;

    // loop through all models and perform frustum culling
    for(auto const& object : models)
    {
        auto plod_node(reinterpret_cast<node::PLodNode*>(object));

        lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());

        auto const& scm_model_matrix = plod_node->get_cached_world_transform();

        // perform frustum culling
        lamure::ren::bvh const* bvh = database->get_model(model_id)->get_bvh();
        scm::gl::frustum const& culling_frustum = cut_update_cam.get_frustum_by_model(math::mat4f(scm_model_matrix));

        std::vector<scm::gl::boxf> const& model_bounding_boxes = bvh->get_bounding_boxes();

        std::unordered_set<lamure::node_t>& nodes_in_frustum = culling_results_per_model[plod_node];

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

        auto const& node_list = cut_map.at(plod_node)->complete_set();

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
}

/////////////////////////////////////////////////////////////////////////////////////////////
void PLodRenderer::set_global_substitution_map(SubstitutionMap const& smap)
{
    for(auto const& pipeline_ptrs : plod_pipelines_)
    {
        for(auto const& pass : *(pipeline_ptrs.second))
        {
            pass->forward_global_substitution_map(smap);
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
void PLodRenderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc, bool render_multiview)
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
    //  sort nodes
    ///////////////////////////////////////////////////////////////////////////
    auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::PLodNode))));

    if(sorted_objects == scene.nodes.end() || sorted_objects->second.empty())
    {
        return; // return if no nodes in scene
    }

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
        return reinterpret_cast<node::PLodNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::PLodNode*>(b)->get_material()->get_shader();
    });

    ///////////////////////////////////////////////////////////////////////////
    // resource initialization
    ///////////////////////////////////////////////////////////////////////////
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();
    _check_for_resource_updates(pipe, ctx);

    ///////////////////////////////////////////////////////////////////////////
    // program initialization
    ///////////////////////////////////////////////////////////////////////////

    target.set_viewport(ctx);

    ///////////////////////////////////////////////////////////////////////////
    // prepare Lamure cut-update
    ///////////////////////////////////////////////////////////////////////////
    lamure::context_t context_id = _register_context_in_cut_update(ctx);

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

    lamure::ren::camera cut_update_cam(lamure_view_id, frustum.get_clip_near(), math::mat4f(frustum.get_view()), math::mat4f(frustum.get_projection()));

    cuts->send_camera(context_id, lamure_view_id, cut_update_cam);
    cuts->send_height_divided_by_top_minus_bottom(context_id, lamure_view_id, height_divided_by_top_minus_bottom);

    auto& gua_depth_buffer = target.get_depth_buffer();

    std::unordered_map<node::PLodNode*, lamure::ren::cut*> cut_map;
    std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>> nodes_in_frustum_per_model;

    for(auto const& object : sorted_objects->second)
    {
        auto plod_node(reinterpret_cast<node::PLodNode*>(object));
        lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());

        auto const& scm_model_matrix = plod_node->get_cached_world_transform();

        cuts->send_transform(context_id, model_id, math::mat4f(scm_model_matrix));
        cuts->send_rendered(context_id, model_id);
        cuts->send_threshold(context_id, model_id, plod_node->get_error_threshold());

        // update current model matrix for LodLibrary in order to make bundle pick work
        database->get_model(model_id)->set_transform(math::mat4f(scm_model_matrix));

        lamure::ren::cut& cut = cuts->get_cut(context_id, lamure_view_id, model_id);
        cut_map.insert(std::make_pair(plod_node, &cut));
    }

    perform_frustum_culling_for_scene(sorted_objects->second, nodes_in_frustum_per_model, cut_map, cut_update_cam, pipe);

    // count splats in cut
#if 0
    std::size_t surfels_in_cut = 0;
    for (auto const& object : sorted_objects->second) {

      auto plod_node(reinterpret_cast<node::PLodNode*>(object));
      lamure::model_t model_id = controller->deduce_model_id(plod_node->get_geometry_description());

      auto bvh = database->get_model(model_id)->get_bvh();
      size_t surfels_per_node_of_model = bvh->get_primitives_per_node();
      
      lamure::ren::cut& cut = cuts->get_cut(context_id, lamure_view_id, model_id);
      for (auto const& node_slot_aggregate : cut.complete_set()) {
        surfels_in_cut += surfels_per_node_of_model;
      }
    }
    std::cout << "Surfels : " << surfels_in_cut << "\n";
#endif

    if(!pipe.current_viewstate().shadow_mode)
    { // normal rendering branch

        PLodPassDescription const& plod_desc = static_cast<PLodPassDescription const&>(desc);

        auto const surfel_render_mode = plod_desc.mode();

        for(auto const& pass : *(plod_pipelines_[surfel_render_mode]))
        {
            pass->render_sub_pass(pipe, desc, shared_pass_resources_[gua_view_id].second, sorted_objects->second, nodes_in_frustum_per_model, context_id, lamure_view_id);
        }
    }
    else
    { // shadow branch
        {
            for(auto const& pass : *(plod_pipelines_[PLodPassDescription::SurfelRenderMode::LQ_ONE_PASS]))
            {
                pass->render_sub_pass(pipe, desc, shared_pass_resources_[gua_view_id].second, sorted_objects->second, nodes_in_frustum_per_model, context_id, lamure_view_id);
            }
        }
    }

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_cpu_query(cpu_query_name_plod_total);
#endif

    // dispatch cut updates
    if(previous_frame_count_ != ctx.framecount)
    {
        previous_frame_count_ = ctx.framecount;
        controller->dispatch(controller->deduce_context_id(ctx.id), ctx.render_device);
    }
}

} // namespace gua
