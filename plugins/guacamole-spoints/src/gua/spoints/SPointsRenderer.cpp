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
#include <gua/spoints/SPointsRenderer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/memory.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/MaterialShader.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/WindowBase.hpp>
#include <gua/spoints/SPointsResource.hpp>
#include <gua/spoints/SPointsNode.hpp>
#include <gua/spoints/spoints_geometry/NetKinectArray.hpp>
#include <gua/spoints/SPointsFeedbackCollector.hpp>

#include <scm/gl_core/render_device/context_guards.h>

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

struct VertexOnly
{
    scm::math::vec3f pos;
};

std::vector<unsigned> proxy_mesh_indices(int size, unsigned width, unsigned height)
{
    std::vector<unsigned> index_array(size);
    unsigned v(0);
    for(unsigned h(0); h < (height - 1); ++h)
    {
        for(unsigned w(0); w < (width - 1); ++w)
        {
            index_array[v] = (w + h * width);
            ++v;
            index_array[v] = (w + h * width + 1);
            ++v;
            index_array[v] = (w + h * width + width);
            ++v;
            index_array[v] = (w + h * width + width);
            ++v;
            index_array[v] = (w + h * width + 1);
            ++v;
            index_array[v] = (w + h * width + 1 + width);
            ++v;
        }
    }
    return index_array;
}

gua::RenderContext::Mesh create_proxy_mesh(gua::RenderContext& ctx, unsigned height_depthimage, unsigned width_depthimage)
{
    int num_vertices = height_depthimage * width_depthimage;
    // int num_indices = height_depthimage * width_depthimage;
    int num_triangles = ((height_depthimage - 1) * (width_depthimage - 1)) * 2;
    int num_triangle_indices = 3 * num_triangles;
    // int num_line_indices = (height_depthimage - 1) * ((width_depthimage - 1) * 3 +
    //                                                   1) + (width_depthimage - 1);

    float step = 1.0f / width_depthimage;

    gua::RenderContext::Mesh proxy_mesh{};
    proxy_mesh.indices_topology = scm::gl::PRIMITIVE_TRIANGLE_LIST;
    proxy_mesh.indices_type = scm::gl::TYPE_UINT;
    proxy_mesh.indices_count = num_triangle_indices;

    proxy_mesh.vertices = ctx.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, num_vertices * sizeof(VertexOnly), 0);

    VertexOnly* data(static_cast<VertexOnly*>(ctx.render_context->map_buffer(proxy_mesh.vertices, scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

    unsigned v(0);
    for(float h = 0.5 * step; h < height_depthimage * step; h += step)
    {
        for(float w = 0.5 * step; w < width_depthimage * step; w += step)
        {
            data[v].pos = scm::math::vec3f(w, h, 0.0f);
            ++v;
        }
    }

    ctx.render_context->unmap_buffer(proxy_mesh.vertices);

    std::vector<unsigned> indices = proxy_mesh_indices(num_triangle_indices, width_depthimage, height_depthimage);

    proxy_mesh.indices = ctx.render_device->create_buffer(scm::gl::BIND_INDEX_BUFFER, scm::gl::USAGE_STATIC_DRAW, num_triangle_indices * sizeof(unsigned int), indices.data());

    proxy_mesh.vertex_array = ctx.render_device->create_vertex_array(scm::gl::vertex_format(0, 0, scm::gl::TYPE_VEC3F, sizeof(VertexOnly)), {proxy_mesh.vertices});
    return proxy_mesh;
    // ctx.render_context->apply(); // necessary ???
}

void draw_proxy_mesh(gua::RenderContext const& ctx, gua::RenderContext::Mesh const& mesh)
{
    scm::gl::context_vertex_input_guard vig(ctx.render_context);
    ctx.render_context->bind_vertex_array(mesh.vertex_array);
    ctx.render_context->bind_index_buffer(mesh.indices, mesh.indices_topology, mesh.indices_type);

    ctx.render_context->apply();
    ctx.render_context->draw_elements(mesh.indices_count);
}

} // namespace

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

SPointsRenderer::SPointsRenderer() : initialized_(false), shaders_loaded_(false), gpu_resources_already_created_(false), current_rendertarget_width_(0), current_rendertarget_height_(0)
{
    _load_shaders();
}

//////////////////////////////////////////////////////////////////////////////
void SPointsRenderer::_load_shaders()
{
    // create stages only with one thread!
    if(!shaders_loaded_)
    {
#ifndef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
#error "This works only with GUACAMOLE_RUNTIME_PROGRAM_COMPILATION enabled"
#endif
        ResourceFactory factory;

        forward_textured_triangles_shader_stages_.clear();
        forward_textured_triangles_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/forward_textured_triangles.vert")));
        forward_textured_triangles_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/forward_textured_triangles.frag")));

        forward_textured_triangles_shader_stages_quantized_.clear();
        forward_textured_triangles_shader_stages_quantized_.push_back(
            ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/forward_textured_triangles_quantized.vert")));
        forward_textured_triangles_shader_stages_quantized_.push_back(
            ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/forward_textured_triangles.frag")));

        /*
              shadow_pass_shader_stages_.clear();
              shadow_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/p01_shadow.vert")));
              shadow_pass_shader_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/p01_shadow.frag")));
        */
        shaders_loaded_ = true;
    }
}

//////////////////////////////////////////////////////////////////////////////
void SPointsRenderer::_initialize_shadow_pass_program()
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
void SPointsRenderer::_create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, bool resize_resource_containers)
{
    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    depth_test_with_writing_depth_stencil_state_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS_EQUAL);

    no_color_accumulation_state_ = ctx.render_device->create_blend_state(false);

    no_backface_culling_rasterizer_state_ =
        ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(false));

    backface_culling_rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_FRONT, scm::gl::ORIENT_CW);

    fullscreen_quad_.reset(new scm::gl::quad_geometry(ctx.render_device, scm::math::vec2(-1.0f, -1.0f), scm::math::vec2(1.0f, 1.0f)));

    // invalidation before first write
    previous_frame_count_ = UINT_MAX;
}

////////////////////////////////////////////////////////////////////////////////
void SPointsRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
    auto& scene = *pipe.current_viewstate().scene;
    auto objects(scene.nodes.find(std::type_index(typeid(node::SPointsNode))));

    if(!(objects != scene.nodes.end() && objects->second.size() > 0))
    {
        return;
    }

    ///////////////////////////////////////////////////////////////////////////
    //  retrieve current view state
    ///////////////////////////////////////////////////////////////////////////

    auto const& camera = pipe.current_viewstate().camera;
    // auto const& frustum = pipe.current_viewstate().frustum;
    auto& target = *pipe.current_viewstate().target;

    auto const& ctx(pipe.get_context());

    int view_id(camera.config.get_view_id());

    std::sort(objects->second.begin(), objects->second.end(), [](node::Node* a, node::Node* b) {
        return reinterpret_cast<node::SPointsNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::SPointsNode*>(b)->get_material()->get_shader();
    });

    if(objects != scene.nodes.end() && objects->second.size() > 0)
    {
        if(!initialized_)
        {
            initialized_ = true;
        }

        scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();

        bool resize_resource_containers = false;
        bool render_resolution_changed = current_rendertarget_width_ != render_target_dims[0] || current_rendertarget_height_ != render_target_dims[1];

        if(!gpu_resources_already_created_ || render_resolution_changed)
        {
            current_rendertarget_width_ = render_target_dims[0];
            current_rendertarget_height_ = render_target_dims[1];
            _create_gpu_resources(ctx, render_target_dims, resize_resource_containers);
            gpu_resources_already_created_ = true;
        }

        ///////////////////////////////////////////////////////////////////////////
        // program initialization
        ///////////////////////////////////////////////////////////////////////////

        {
            scm::gl::context_all_guard context_guard(ctx.render_context);

            for(auto& o : objects->second)
            {
                auto spoints_node(reinterpret_cast<node::SPointsNode*>(o));
                auto spoints_desc(spoints_node->get_spoints_description());

                if(true)
                {
                    // std::cout << "IS SERVER RESOURCE\n";

                    auto const& cached_model_matrix(spoints_node->get_cached_world_transform());
                    auto normal_matrix(scm::math::transpose(scm::math::inverse(spoints_node->get_cached_world_transform())));
                    auto view_matrix(pipe.current_viewstate().frustum.get_view());

                    scm::math::mat4f model_matrix = scm::math::mat4f(cached_model_matrix);
                    scm::math::mat4f mv_matrix = scm::math::mat4f(view_matrix) * scm::math::mat4f(model_matrix);
                    scm::math::mat4f projection_matrix = scm::math::mat4f(pipe.current_viewstate().frustum.get_projection());
                    scm::math::mat4f viewprojection_matrix = projection_matrix * scm::math::mat4f(view_matrix);

                    spoints::matrix_package current_package;

                    memcpy((char*)&current_package, (char*)model_matrix.data_array, 16 * sizeof(float));
                    memcpy((char*)&current_package + 16 * sizeof(float), (char*)viewprojection_matrix.data_array, 16 * sizeof(float));
                    memcpy((char*)&current_package + 32 * sizeof(float), (char*)mv_matrix.data_array, 16 * sizeof(float));
                    memcpy(((char*)&current_package) + 48 * sizeof(float), (char*)projection_matrix.data_array, 16 * sizeof(float));

                    current_package.res_xy[0] = render_target_dims.x;
                    current_package.res_xy[1] = render_target_dims.y;

                    auto const& camera = pipe.current_viewstate().camera;
                    auto const camera_view_id = camera.config.view_id();

                    uint32_t current_camera_feedback_uuid = camera.config.view_id();

                    if(camera.config.enable_stereo())
                    {
                        bool is_left_cam = true;
                        if(camera_view_id == last_rendered_view_id)
                        {
                            last_rendered_side = (last_rendered_side + 1) % 2;
                            is_left_cam = (last_rendered_side == 0) ? true : false;
                        }

                        current_package.camera_type = (is_left_cam ? 1 : 2);
                    }
                    else
                    {
                        last_rendered_side = 0;
                        current_package.camera_type = 0;
                    }

                    last_rendered_view_id = camera_view_id;

                    current_package.uuid = current_camera_feedback_uuid;

                    auto spoints_resource = std::static_pointer_cast<SPointsResource>(GeometryDatabase::instance()->lookup(spoints_desc));
                    if(!spoints_resource)
                    {
                        gua::Logger::LOG_WARNING << "SPointsRenderer::draw(): Invalid spoints." << std::endl;
                        continue;
                    }

                    // spoints_resource->push_matrix_package(cm_package);
                    std::string feedback_socket_string_of_resource = spoints_resource->get_socket_string();

                    if(!spoints_resource->has_calibration(ctx))
                    {
                        current_package.calibration_request = true;
                    }
                    else
                    {
                        current_package.calibration_request = false;
                    }

                    if("" != feedback_socket_string_of_resource)
                    {
                        SPointsFeedbackCollector::instance()->push_feedback_matrix(ctx, feedback_socket_string_of_resource, current_package);
                        //      std::cout << "Feedback socket string was not 0, but: " << feedback_socket_string_of_resource << "\n";
                    }
                    else
                    {
                        //     std::cout << "FBS: 0" << "\n";
                    }
                }
            }

            //////////////////////////////////////////////////////////////////////////
            // TRI PASS: TEXTURES TRI PASS:
            //////////////////////////////////////////////////////////////////////////

            {
                scm::gl::context_all_guard context_guard(ctx.render_context);

                bool write_depth = true;
                target.bind(ctx, write_depth);
                target.set_viewport(ctx);
                // forward_textured_triangles_pass_program_->use(ctx);

                ctx.render_context->set_depth_stencil_state(depth_test_with_writing_depth_stencil_state_);
                ctx.render_context->set_blend_state(no_color_accumulation_state_);
                ctx.render_context->apply();

                for(auto& o : objects->second)
                {
                    auto spoints_node(reinterpret_cast<node::SPointsNode*>(o));

                    auto spoints_desc(spoints_node->get_spoints_description());

                    if(!GeometryDatabase::instance()->contains(spoints_desc))
                    {
                        gua::Logger::LOG_WARNING << "SPointsRenderer::draw(): No such spoints." << spoints_desc << ", " << std::endl;
                        continue;
                    }

                    auto spoints_resource = std::static_pointer_cast<SPointsResource>(GeometryDatabase::instance()->lookup(spoints_desc));
                    if(!spoints_resource)
                    {
                        gua::Logger::LOG_WARNING << "SPointsRenderer::draw(): Invalid spoints." << std::endl;
                        continue;
                    }

                    spoints_resource->update_buffers(pipe.get_context(), pipe);

                    // get material dependent shader
                    std::shared_ptr<ShaderProgram> current_shader;

                    if(!(spoints_resource->is_vertex_data_fully_encoded()))
                    {
                        // is_vertex_data_fully_encoded
                        MaterialShader* current_material = spoints_node->get_material()->get_shader();
                        if(current_material)
                        {
                            auto shader_iterator = forward_textured_triangles_pass_programs_quantized_.find(current_material);
                            if(shader_iterator != forward_textured_triangles_pass_programs_quantized_.end())
                            {
                                current_shader = shader_iterator->second;
                            }
                            else
                            {
                                auto smap = global_substitution_map_;
                                for(const auto& i : current_material->generate_substitution_map())
                                    smap[i.first] = i.second;

                                current_shader = std::make_shared<ShaderProgram>();
                                current_shader->set_shaders(

                                forward_textured_triangles_shader_stages_quantized_, std::list<std::string>(), false, smap);
                                forward_textured_triangles_pass_programs_quantized_[current_material] = current_shader;
                            }
                        }
                        else
                        {
                            Logger::LOG_WARNING << "SPointsPass::render(): Cannot find material: " << spoints_node->get_material()->get_shader_name() << std::endl;
                        }
                    }
                    else
                    {
                        MaterialShader* current_material = spoints_node->get_material()->get_shader();
                        if(current_material)
                        {
                            auto shader_iterator = forward_textured_triangles_pass_programs_.find(current_material);
                            if(shader_iterator != forward_textured_triangles_pass_programs_.end())
                            {
                                current_shader = shader_iterator->second;
                            }
                            else
                            {
                                auto smap = global_substitution_map_;
                                for(const auto& i : current_material->generate_substitution_map())
                                    smap[i.first] = i.second;

                                current_shader = std::make_shared<ShaderProgram>();
                                current_shader->set_shaders(

                                forward_textured_triangles_shader_stages_, std::list<std::string>(), false, smap);
                                forward_textured_triangles_pass_programs_[current_material] = current_shader;
                            }
                        }
                        else
                        {
                            Logger::LOG_WARNING << "SPointsPass::render(): Cannot find material: " << spoints_node->get_material()->get_shader_name() << std::endl;
                        }
                    }

                    scm::gl::context_all_guard context_guard(ctx.render_context);

                    bool write_depth = true;
                    target.bind(ctx, write_depth);

                    current_shader->use(ctx);

                    if(!(spoints_resource->is_vertex_data_fully_encoded())) {
                        ctx.render_context->set_rasterizer_state(backface_culling_rasterizer_state_);
                    } else {
                        ctx.render_context->set_rasterizer_state(no_backface_culling_rasterizer_state_);
                    } 


                    auto const& model_matrix(spoints_node->get_cached_world_transform());
                    auto normal_matrix(scm::math::transpose(scm::math::inverse(spoints_node->get_cached_world_transform())));
                    auto view_matrix(pipe.current_viewstate().frustum.get_view());

                    scm::math::mat4f mv_matrix = scm::math::mat4f(view_matrix) * scm::math::mat4f(model_matrix);

                    scm::math::mat4f projection_matrix = scm::math::mat4f(pipe.current_viewstate().frustum.get_projection());

                    scm::math::mat4f mvp_matrix = projection_matrix * mv_matrix;

                    int rendering_mode = pipe.current_viewstate().shadow_mode ? (spoints_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

                    if(current_shader)
                    {
                        current_shader->use(ctx);
                        current_shader->set_uniform(ctx,
                                                    math::vec2ui(target.get_width(), target.get_height()),
                                                    "gua_resolution"); // TODO: pass gua_resolution. Probably should be somehow else implemented
                        current_shader->set_uniform(ctx, 1.0f / target.get_width(), "gua_texel_width");
                        current_shader->set_uniform(ctx, 1.0f / target.get_height(), "gua_texel_height");
                        // hack
                        current_shader->set_uniform(ctx, ::get_handle(target.get_depth_buffer()), "gua_gbuffer_depth");
                        current_shader->set_uniform(ctx, scm::math::mat4f(model_matrix), "kinect_model_matrix");

                        current_shader->set_uniform(ctx, scm::math::mat4f(mv_matrix), "kinect_mv_matrix");

                        current_shader->set_uniform(ctx, scm::math::mat4f(mvp_matrix), "kinect_mvp_matrix");

                        current_shader->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);
                    }

                    spoints_resource->draw_textured_triangle_soup(ctx, current_shader);

                    current_shader->unuse(ctx);
                }

                target.unbind(ctx);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
