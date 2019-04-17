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
#include <gua/renderer/TV_3Renderer.hpp>
//#include <gua/renderer/TV_3SurfacePass.hpp>
//#include <gua/renderer/TV_3VolumePass.hpp>

// guacamole headers
#include <gua/renderer/TV_3Resource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/TV_3Node.hpp>
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/View.hpp>

#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>

#include <gua/renderer/Window.hpp>

#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/texture_objects/texture_image.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/render_device.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>
#include <boost/assign/list_of.hpp>

namespace gua
{
//////////////////////////////////////////////////////////////////////////////
TV_3Renderer::TV_3Renderer(gua::RenderContext const& ctx, gua::SubstitutionMap const& substitution_map)
    : shaders_loaded_(false), forward_cube_shader_program_(nullptr), compositing_shader_program_(nullptr), no_backface_culling_rasterizer_state_(nullptr), frontface_culling_rasterizer_state_(nullptr)
{
    /*
                                     global_substitution_map_uncompressed_["gua_tv_3_sampler_3d_type"] = "sampler3D";
                                     global_substitution_map_compressed_["gua_tv_3_sampler_3d_type"] = "usampler3D";
    */

    // global_substitution_maps_
    std::vector<TV_3Resource::CompressionMode> compression_modes = {TV_3Resource::CompressionMode::UNCOMPRESSED, TV_3Resource::CompressionMode::SW_VQ, TV_3Resource::CompressionMode::SW_HVQ};
    std::vector<node::TV_3Node::SpatialFilterMode> s_filtering_modes = {node::TV_3Node::SpatialFilterMode::S_NEAREST, node::TV_3Node::SpatialFilterMode::S_LINEAR};
    std::vector<node::TV_3Node::TemporalFilterMode> t_filtering_modes = {node::TV_3Node::TemporalFilterMode::T_NEAREST}; //, TemporalFilterMode::T_LINEAR};
    std::vector<node::TV_3Node::RenderMode> render_modes = {node::TV_3Node::RenderMode::VOL_ISOSURFACE,
                                                            node::TV_3Node::RenderMode::VOL_MAX_INTENSITY,
                                                            node::TV_3Node::RenderMode::VOL_COMPOSITING,
                                                            node::TV_3Node::RenderMode::VOL_AVG_INTENSITY,
                                                            node::TV_3Node::RenderMode::SUR_PBR}; //, TemporalFilterMode::T_LINEAR};

    // set shader compilation flags according to compiled shader
    for(auto const c_mode : compression_modes)
    {
        for(auto const sf_mode : s_filtering_modes)
        {
            for(auto const tf_mode : t_filtering_modes)
            {
                for(auto const r_mode : render_modes)
                {
                    std::string gua_tv_3_uncompressed_string = "0";
                    std::string gua_tv_3_vq_compressed_string = "0";
                    if(TV_3Resource::CompressionMode::UNCOMPRESSED == c_mode)
                    {
                        gua_tv_3_uncompressed_string = "1";
                    }
                    else
                    {
                        gua_tv_3_vq_compressed_string = "1";
                    }

                    std::string gua_tv_3_spatially_nearest_filter_string = "0";
                    std::string gua_tv_3_spatially_linear_filter_string = "0";
                    if(node::TV_3Node::SpatialFilterMode::S_NEAREST == sf_mode)
                    {
                        gua_tv_3_spatially_nearest_filter_string = "1";
                    }
                    else
                    {
                        gua_tv_3_spatially_linear_filter_string = "1";
                    }

                    std::string gua_tv_3_surface_pbr_behaviour_string = "0";
                    std::string gua_tv_3_volume_behaviour_string = "0";

                    std::string gua_tv_3_mode_vol_isosurface_string = "0";
                    std::string gua_tv_3_mode_vol_max_intensity_string = "0";
                    std::string gua_tv_3_mode_vol_compositing_string = "0";
                    std::string gua_tv_3_mode_vol_avg_intensity_string = "0";

                    if(node::TV_3Node::RenderMode::SUR_PBR == r_mode)
                    {
                        gua_tv_3_surface_pbr_behaviour_string = "1";
                    }
                    else
                    {
                        gua_tv_3_volume_behaviour_string = "1";
                        if(node::TV_3Node::RenderMode::VOL_ISOSURFACE == r_mode)
                        {
                            gua_tv_3_mode_vol_isosurface_string = "1";
                        }
                        else if(node::TV_3Node::RenderMode::VOL_MAX_INTENSITY == r_mode)
                        {
                            gua_tv_3_mode_vol_max_intensity_string = "1";
                        }
                        else if(node::TV_3Node::RenderMode::VOL_COMPOSITING == r_mode)
                        {
                            gua_tv_3_mode_vol_compositing_string = "1";
                        }
                        else if(node::TV_3Node::RenderMode::VOL_AVG_INTENSITY == r_mode)
                        {
                            gua_tv_3_mode_vol_avg_intensity_string = "1";
                        }
                    }

                    // copy substitution map from other passes
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode] = substitution_map;

                    // add volume rendering specific variables
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_uncompressed"] = gua_tv_3_uncompressed_string;
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_vq_compressed"] = gua_tv_3_vq_compressed_string;

                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_spatially_nearest_filter"] = gua_tv_3_spatially_nearest_filter_string;
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_spatially_linear_filter"] = gua_tv_3_spatially_linear_filter_string;

                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_surface_pbr_behaviour"] = gua_tv_3_surface_pbr_behaviour_string;
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_volume_behaviour"] = gua_tv_3_volume_behaviour_string;

                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_mode_vol_isosurface"] = gua_tv_3_mode_vol_isosurface_string;
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_mode_vol_max_intensity"] = gua_tv_3_mode_vol_max_intensity_string;
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_mode_vol_compositing"] = gua_tv_3_mode_vol_compositing_string;
                    global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode]["gua_tv_3_mode_vol_avg_intensity"] = gua_tv_3_mode_vol_avg_intensity_string;
                }
            }
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////
void TV_3Renderer::_load_shaders()
{
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
    ResourceFactory factory;
    ray_casting_program_stages_.clear();
    ray_casting_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/tv_3/ray_casting.vert")));
    ray_casting_program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/tv_3/ray_casting.frag")));

    shaders_loaded_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void TV_3Renderer::_initialize_ray_casting_program(
    MaterialShader* material, CompressionMode const c_mode, SpatialFilterMode const sf_mode, TemporalFilterMode const tf_mode, NodeRenderMode const r_mode)
{
    auto& current_map_by_mode = ray_casting_programs_[c_mode][sf_mode][tf_mode][r_mode];
    if(!current_map_by_mode.count(material))
    {
        auto program = std::make_shared<ShaderProgram>();

        auto smap = global_substitution_maps_[c_mode][sf_mode][tf_mode][r_mode];

        for(const auto& i : material->generate_substitution_map())
        {
            smap[i.first] = i.second;
        }

        program->set_shaders(ray_casting_program_stages_, std::list<std::string>(), false, smap);
        current_map_by_mode[material] = program;
    }
    assert(current_map_by_mode.count(material));
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<ShaderProgram> TV_3Renderer::_get_material_program(MaterialShader* material,
                                                                   std::shared_ptr<ShaderProgram> const& current_program,
                                                                   bool& program_changed,
                                                                   CompressionMode const c_mode,
                                                                   SpatialFilterMode const sf_mode,
                                                                   TemporalFilterMode const tf_mode,
                                                                   NodeRenderMode const r_mode)
{
    auto& current_map_by_mode = ray_casting_programs_[c_mode][sf_mode][tf_mode][r_mode];
    auto shader_iterator = current_map_by_mode.find(material);
    if(shader_iterator == current_map_by_mode.end())
    {
        try
        {
            _initialize_ray_casting_program(material, c_mode, sf_mode, tf_mode, r_mode);
            program_changed = true;
            return current_map_by_mode.at(material);
        }
        catch(std::exception& e)
        {
            Logger::LOG_WARNING << "TV_3Pass::_get_material_program(): Cannot create material for raycasting program: " << e.what() << std::endl;
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
void TV_3Renderer::_create_fbo_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims) {}

///////////////////////////////////////////////////////////////////////////////
void TV_3Renderer::_create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims)
{
    // invalidation before first write
    previous_frame_count_ = UINT_MAX;

    _create_fbo_resources(ctx, render_target_dims);

    if(!fullscreen_quad_)
    {
        fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f)));
    }

    no_backface_culling_rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true);

    frontface_culling_rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT, scm::gl::ORIENT_CCW, true);

    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    trilin_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

    // box_vertex_array_;
    box_vertex_buffer_ = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, 36 * 3 * sizeof(float));

    scm::gl::render_context_ptr main_ctx = ctx.render_device->main_context();

    static const float g_vertex_buffer_data[] = {0.0f, 0.0f, 0.0f, // triangle 1 : begin
                                                 0.0f, 1.0f, 1.0f, // triangle 1 : end
                                                 0.0f, 0.0f, 1.0f,

                                                 1.0f, 1.0f, 0.0f, // triangle 2 : begin
                                                 0.0f, 1.0f, 0.0f, // triangle 2 : end
                                                 0.0f, 0.0f, 0.0f,

                                                 1.0f, 0.0f, 1.0f, 1.0f, 0.0f,
                                                 0.0f, 0.0f, 0.0f, 0.0f,

                                                 1.0f, 1.0f, 0.0f, 0.0f, 0.0f,
                                                 0.0f, 1.0f, 0.0f, 0.0f,

                                                 0.0f, 0.0f, 0.0f, 0.0f, 1.0f,
                                                 0.0f, 0.0f, 1.0f, 1.0f,

                                                 1.0f, 0.0f, 1.0f, 0.0f, 0.0f,
                                                 0.0f, 0.0f, 0.0f, 1.0f,

                                                 0.0f, 1.0f, 1.0f, 1.0f, 0.0f,
                                                 1.0f, 0.0f, 0.0f, 1.0f,

                                                 1.0f, 1.0f, 1.0f, 1.0f, 1.0f,
                                                 0.0f, 1.0f, 0.0f, 0.0f,

                                                 1.0f, 0.0f, 0.0f, 1.0f, 0.0f,
                                                 1.0f, 1.0f, 1.0f, 1.0f,

                                                 1.0f, 1.0f, 1.0f, 0.0f, 1.0f,
                                                 0.0f, 1.0f, 1.0f, 0.0f,

                                                 1.0f, 1.0f, 1.0f, 0.0f, 1.0f,
                                                 1.0f, 0.0f, 1.0f, 0.0f,

                                                 1.0f, 1.0f, 1.0f, 1.0f, 0.0f,
                                                 1.0f, 0.0f, 1.0f, 1.0f

    };

    float* mapped_vertex_buffer = static_cast<float*>(main_ctx->map_buffer(box_vertex_buffer_, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));

    for(int i = 0; i < 36 * 3; ++i)
    {
        mapped_vertex_buffer[i] = g_vertex_buffer_data[i];
    }

    main_ctx->unmap_buffer(box_vertex_buffer_);

    box_vertex_array_ = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, 3 * sizeof(float)), boost::assign::list_of(box_vertex_buffer_));

    /*
        box_element_buffer_ = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, 14 * sizeof(uint32_t));
        uint32_t* mapped_element_buffer = static_cast<uint32_t*>(main_ctx->map_buffer(box_element_buffer_, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER));
        mapped_element_buffer[ 0] = 3;
        mapped_element_buffer[ 1] = 2;
        mapped_element_buffer[ 2] = 6;
        mapped_element_buffer[ 3] = 7;
        mapped_element_buffer[ 4] = 4;
        mapped_element_buffer[ 5] = 2;
        mapped_element_buffer[ 6] = 0;
        mapped_element_buffer[ 7] = 3;
        mapped_element_buffer[ 8] = 1;
        mapped_element_buffer[ 9] = 6;
        mapped_element_buffer[10] = 5;
        mapped_element_buffer[11] = 4;
        mapped_element_buffer[12] = 1;
        mapped_element_buffer[13] = 0;

        main_ctx->unmap_buffer(box_element_buffer_);
    */

    /*
        resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_NORMAL_RESULT] = ctx.render_device
          ->create_texture_2d(render_target_dims,
                              scm::gl::FORMAT_RGB_16F,
                              1, 1, 1);

        resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_PBR_RESULT] = ctx.render_device
          ->create_texture_2d(render_target_dims,
                              scm::gl::FORMAT_RGB_16F,
                              1, 1, 1);

        resource_ptrs_.attachments_[plod_shared_resources::AttachmentID::ACCUM_PASS_WEIGHT_AND_DEPTH_RESULT] = ctx.render_device
          ->create_texture_2d(render_target_dims,
                              scm::gl::FORMAT_RG_32F,
                              1, 1, 1);

        depth_test_without_writing_depth_stencil_state_ = ctx.render_device
          ->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS_EQUAL);

        color_accumulation_state_ = ctx.render_device->create_blend_state(true,
                                                                          scm::gl::FUNC_ONE,
                                                                          scm::gl::FUNC_ONE,
                                                                          scm::gl::FUNC_ONE,
                                                                          scm::gl::FUNC_ONE,
                                                                          scm::gl::EQ_FUNC_ADD,
                                                                          scm::gl::EQ_FUNC_ADD);

        no_backface_culling_rasterizer_state_ = ctx.render_device
          ->create_rasterizer_state(scm::gl::FILL_SOLID,
                                    scm::gl::CULL_NONE,
                                    scm::gl::ORIENT_CCW,
                                    false,
                                    false,
                                    0.0,
                                    false,
                                    false,
                                    scm::gl::point_raster_state(false));

        _register_shared_resources(shared_resources);
    */
}

/////////////////////////////////////////////////////////////////////////////////////////////
void TV_3Renderer::_check_for_resource_updates(gua::Pipeline const& pipe, RenderContext const& ctx)
{
    // get current unique view id and resolution
    auto const& camera = pipe.current_viewstate().camera;
    scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

    // check if resources for this view and resolution are already available
    bool resolution_available = false;

    resolution_available = (current_rendertarget_dims_ == render_target_dims);

    // if not, allocate
    if(!resolution_available)
    {
        _create_gpu_resources(ctx, render_target_dims);

        current_rendertarget_dims_ = render_target_dims;
    }
}

///////////////////////////////////////////////////////////////////////////////
void TV_3Renderer::render(gua::Pipeline& pipe, PipelinePassDescription const& desc)
{
    RenderContext const& ctx(pipe.get_context());

    ///////////////////////////////////////////////////////////////////////////
    //  retrieve current view state
    ///////////////////////////////////////////////////////////////////////////
    auto& scene = *pipe.current_viewstate().scene;
    auto const& camera = pipe.current_viewstate().camera;
    auto const& frustum = pipe.current_viewstate().frustum;

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    std::string cpu_query_name_plod_total = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / LodPass";
    pipe.begin_cpu_query(cpu_query_name_plod_total);
#endif

    ///////////////////////////////////////////////////////////////////////////
    //  sort nodes
    ///////////////////////////////////////////////////////////////////////////
    auto sorted_objects(scene.nodes.find(std::type_index(typeid(node::TV_3Node))));

    if(sorted_objects == scene.nodes.end() || sorted_objects->second.empty())
    {
        return; // return if no nodes in scene
    }

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b) {
        return reinterpret_cast<node::TV_3Node*>(a)->get_material()->get_shader() < reinterpret_cast<node::TV_3Node*>(b)->get_material()->get_shader();
    });

    ///////////////////////////////////////////////////////////////////////////
    // resource initialization
    ///////////////////////////////////////////////////////////////////////////
    _check_for_resource_updates(pipe, ctx);

    _clear_fbo_attachments(ctx);

    if(!shaders_loaded_)
    {
        _load_shaders();
    }

    ///////////////////////////////////////////////////////////////////////////
    // actual volume rendering pass
    ///////////////////////////////////////////////////////////////////////////
    _raycasting_pass(pipe, sorted_objects->second, desc);

    _postprocessing_pass(pipe, desc);

#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    pipe.end_cpu_query(cpu_query_name_plod_total);
#endif

    // dispatch cut updates
    if(previous_frame_count_ != ctx.framecount)
    {
        previous_frame_count_ = ctx.framecount;
    }
}

} // namespace gua
