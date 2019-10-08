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
#include <gua/video3d/Video3DRenderer.hpp>

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
#include <gua/video3d/Video3DResource.hpp>
#include <gua/video3d/Video3DNode.hpp>
#include <gua/video3d/video3d_geometry/NetKinectArray.hpp>

#include <scm/gl_core/render_device/context_guards.h>

namespace
{
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
Video3DRenderer::Video3DData::Video3DData(RenderContext const& ctx, Video3DResource const& video3d_ressource)
{
    // initialize Texture Arrays (kinect depths & colors)
    depth_tex_ = ctx.render_device->create_texture_2d(
        scm::math::vec2ui(video3d_ressource.width_depthimage(), video3d_ressource.height_depthimage()), scm::gl::FORMAT_R_32F, 0, video3d_ressource.number_of_cameras(), 1);

    depth_tex_processed_ = ctx.render_device->create_texture_2d(
        scm::math::vec2ui(video3d_ressource.width_depthimage(), video3d_ressource.height_depthimage()), scm::gl::FORMAT_RG_32F, 0, video3d_ressource.number_of_cameras(), 1);

    color_tex_ = ctx.render_device->create_texture_2d(scm::math::vec2ui(video3d_ressource.width_colorimage(), video3d_ressource.height_colorimage()),
                                                      video3d_ressource.calib_files()[0]->isCompressedRGB() ? scm::gl::FORMAT_BC1_RGBA : scm::gl::FORMAT_RGB_8,
                                                      0,
                                                      video3d_ressource.number_of_cameras(),
                                                      1);

    rstate_solid_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true);

    nka_ = std::make_shared<video3d::NetKinectArray>(video3d_ressource.calib_files(), video3d_ressource.server_endpoint(), video3d_ressource.color_size(), video3d_ressource.depth_size_byte());

    // generate and download calibvolumes for this context
    for(auto const& calib : video3d_ressource.calib_files())
    {
        std::vector<void*> raw_cv_xyz;
        raw_cv_xyz.push_back(calib->cv_xyz);
        // should be: glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB32F, cv_width, cv_height,
        // cv_depth, 0, GL_RGB, GL_FLOAT, (unsigned char*) cv_xyz);
        cv_xyz_.push_back(ctx.render_device->create_texture_3d(scm::math::vec3ui(calib->cv_width, calib->cv_height, calib->cv_depth), scm::gl::FORMAT_RGB_32F, 0, scm::gl::FORMAT_RGB_32F, raw_cv_xyz));

        std::vector<void*> raw_cv_uv;
        raw_cv_uv.push_back(calib->cv_uv);
        // should be: glTexImage3D(GL_TEXTURE_3D, 0, GL_RG32F, cv_width, cv_height,
        // cv_depth, 0, GL_RG, GL_FLOAT, (unsigned char*) cv_uv);
        cv_uv_.push_back(ctx.render_device->create_texture_3d(scm::math::vec3ui(calib->cv_width, calib->cv_height, calib->cv_depth), scm::gl::FORMAT_RG_32F, 0, scm::gl::FORMAT_RG_32F, raw_cv_uv));
    }

    frame_counter_ = 0;
}

////////////////////////////////////////////////////////////////////////////////

Video3DRenderer::Video3DRenderer() : initialized_(false)
{
    ResourceFactory factory;

    // create depth shader
    std::vector<ShaderProgramStage> warp_pass_stages;
    warp_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/warp_pass.vert")));
    warp_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, factory.read_shader_file("resources/shaders/warp_pass.geom")));
    warp_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/warp_pass.frag")));

    warp_pass_program_ = std::make_shared<ShaderProgram>();
    warp_pass_program_->set_shaders(warp_pass_stages);

    // create final shader description
    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/blend_pass.vert")));
    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/blend_pass.frag")));

    // create depth process shader
    std::vector<ShaderProgramStage> depth_process_stages;
    depth_process_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, factory.read_shader_file("resources/shaders/common/fullscreen_quad.vert")));
    depth_process_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, factory.read_shader_file("resources/shaders/depth_process.frag")));

    depth_process_program_ = std::make_shared<ShaderProgram>();
    depth_process_program_->set_shaders(depth_process_stages);
}

////////////////////////////////////////////////////////////////////////////////
void Video3DRenderer::draw_video3dResource(RenderContext& ctx, Video3DResource const& video3d_ressource)
{
    // ctx.render_context->apply();
    auto iter = ctx.meshes.find(video3d_ressource.uuid());
    if(iter == ctx.meshes.end())
    {
        ctx.meshes[video3d_ressource.uuid()] = create_proxy_mesh(ctx, video3d_ressource.height_depthimage(), video3d_ressource.width_depthimage());
        draw_proxy_mesh(ctx, ctx.meshes[video3d_ressource.uuid()]);
    }
    else
    {
        draw_proxy_mesh(ctx, iter->second);
    }
}

////////////////////////////////////////////////////////////////////////////////
void Video3DRenderer::update_buffers(RenderContext const& ctx, Video3DResource const& video3d_ressource, Pipeline& pipe)
{
    auto iter = video3Ddata_.find(video3d_ressource.uuid());
    if(iter == video3Ddata_.end())
    {
        video3Ddata_[video3d_ressource.uuid()] = Video3DData(ctx, video3d_ressource);
    }

    Video3DData& video3d_data = video3Ddata_[video3d_ressource.uuid()];

    if(video3d_data.frame_counter_ != ctx.framecount)
    {
        video3d_data.frame_counter_ = ctx.framecount;
    }
    else
    {
        return;
    }

    if(video3d_data.nka_->update())
    {
        unsigned char* buff = video3d_data.nka_->getBuffer();
        for(unsigned i = 0; i < video3d_ressource.number_of_cameras(); ++i)
        {
            ctx.render_context->update_sub_texture(video3d_data.color_tex_,
                                                   scm::gl::texture_region(scm::math::vec3ui(0, 0, i), scm::math::vec3ui(video3d_data.color_tex_->dimensions(), 1)),
                                                   0, // mip-mapping level
                                                   video3d_ressource.calib_files()[0]->isCompressedRGB() ? scm::gl::FORMAT_BC1_RGBA : scm::gl::FORMAT_RGB_8,
                                                   static_cast<void*>(buff));
            buff += video3d_ressource.color_size();
            ctx.render_context->update_sub_texture(video3d_data.depth_tex_,
                                                   scm::gl::texture_region(scm::math::vec3ui(0, 0, i), scm::math::vec3ui(video3d_data.depth_tex_->dimensions(), 1)),
                                                   0, // mip-mapping level
                                                   scm::gl::FORMAT_R_32F,
                                                   static_cast<void*>(buff));
            buff += video3d_ressource.depth_size_byte();
        }

        process_textures(ctx, video3d_ressource, pipe);
    }
}

////////////////////////////////////////////////////////////////////////////////
void Video3DRenderer::process_textures(RenderContext const& ctx, Video3DResource const& video3d_ressource, Pipeline& pipe)
{
    Video3DData& video3d_data = video3Ddata_[video3d_ressource.uuid()];
    // store current state
    scm::gl::context_all_guard all_guard(ctx.render_context);

    ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), scm::math::vec3ui(video3d_data.depth_tex_->dimensions())));
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_tex_process_);
    ctx.render_context->set_frame_buffer(fbo_depth_process_);

    depth_process_program_->use(ctx);
    // bind depth texture to unit
    ctx.render_context->bind_texture(video3d_data.depth_tex_, nearest_sampler_state_, 1);

    for(unsigned i = 0; i < video3d_ressource.number_of_cameras(); ++i)
    {
        fbo_depth_process_->attach_color_buffer(0, video3d_data.depth_tex_processed_, 0, i);

        depth_process_program_->set_uniform(ctx, int(i), "layer");
        depth_process_program_->set_uniform(ctx, video3d_ressource.calibration_file(i).getTexSizeInvD(), "tex_size_inv");
        depth_process_program_->set_uniform(ctx, video3d_ressource.calibration_file(i).cv_min_d, "cv_min_d");
        depth_process_program_->set_uniform(ctx, video3d_ressource.calibration_file(i).cv_max_d, "cv_max_d");

        // render to fbo
        pipe.draw_quad();
    }

    depth_process_program_->unuse(ctx);
}

////////////////////////////////////////////////////////////////////////////////
void Video3DRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{
    ///////////////////////////////////////////////////////////////////////////
    //  retrieve current view state
    ///////////////////////////////////////////////////////////////////////////
    auto& scene = *pipe.current_viewstate().scene;
    auto const& camera = pipe.current_viewstate().camera;
    // auto const& frustum = pipe.current_viewstate().frustum;
    auto& target = *pipe.current_viewstate().target;

    auto const& ctx(pipe.get_context());

    if(!initialized_)
    {
        initialized_ = true;

        warp_pass_program_->upload_to(ctx);
        depth_process_program_->upload_to(ctx);
        // TODO:::::
        // if (warp_color_result_) {
        //   return upload_succeeded;
        // }
        // else {
        //     // continue instantiation below
        // }

        // initialize Texture Arrays (kinect depths & colors)
        warp_color_result_ = ctx.render_device->create_texture_2d(camera.config.resolution(), scm::gl::FORMAT_RGBA_32F, 1, MAX_NUM_KINECTS, 1);

        warp_depth_result_ = ctx.render_device->create_texture_2d(camera.config.resolution(), scm::gl::FORMAT_D32F, 1, MAX_NUM_KINECTS, 1);

        warp_result_fbo_ = ctx.render_device->create_frame_buffer();
        linear_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
        nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
        depth_stencil_state_warp_pass_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
        depth_stencil_state_blend_pass_ = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
        no_bfc_rasterizer_state_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);

        warp_pass_program_->get_program()->uniform_sampler("depth_video3d_texture", 0);
        warp_pass_program_->get_program()->uniform_sampler("cv_xyz", 1);
        warp_pass_program_->get_program()->uniform_sampler("cv_uv", 2);

        fbo_depth_process_ = ctx.render_device->create_frame_buffer();
        depth_stencil_state_tex_process_ = ctx.render_device->create_depth_stencil_state(false, false);
        // input texture bound to unit 1
        depth_process_program_->get_program()->uniform_sampler("depth_texture", 1);
    }

    auto objects(scene.nodes.find(std::type_index(typeid(node::Video3DNode))));
    int view_id(camera.config.get_view_id());

    if(objects != scene.nodes.end() && objects->second.size() > 0)
    {
        for(auto& o : objects->second)
        {
            auto video_node(reinterpret_cast<node::Video3DNode*>(o));
            auto video_desc(video_node->get_video_description());

            if(!GeometryDatabase::instance()->contains(video_desc))
            {
                gua::Logger::LOG_WARNING << "Video3DRenderer::draw(): No such video." << video_desc << ", " << std::endl;
                continue;
            }

            auto video3d_ressource = std::static_pointer_cast<Video3DResource>(GeometryDatabase::instance()->lookup(video_desc));
            if(!video3d_ressource)
            {
                gua::Logger::LOG_WARNING << "Video3DRenderer::draw(): Invalid video." << std::endl;
                continue;
            }

            // update stream data
            update_buffers(pipe.get_context(), *video3d_ressource, pipe);
            auto const& video3d_data = video3Ddata_[video3d_ressource->uuid()];

            auto const& model_matrix(video_node->get_cached_world_transform());
            auto normal_matrix(scm::math::transpose(scm::math::inverse(video_node->get_cached_world_transform())));
            auto view_matrix(pipe.current_viewstate().frustum.get_view());
            const float scaling = scm::math::length((model_matrix * view_matrix) * scm::math::vec4d(1.0, 0.0, 0.0, 0.0));
            {
                // single texture only
                scm::gl::context_all_guard guard(ctx.render_context);

                ctx.render_context->set_rasterizer_state(no_bfc_rasterizer_state_);
                ctx.render_context->set_depth_stencil_state(depth_stencil_state_warp_pass_);

                // set uniforms
                ctx.render_context->bind_texture(video3d_data.depth_tex_processed_, nearest_sampler_state_, 0);

                warp_pass_program_->apply_uniform(ctx, "gua_normal_matrix", math::mat4f(normal_matrix));
                warp_pass_program_->apply_uniform(ctx, "gua_model_matrix", math::mat4f(model_matrix));
                warp_pass_program_->set_uniform(ctx, int(1), "bbxclip");

                auto const& bbox(video3d_ressource->get_bounding_box());
                warp_pass_program_->set_uniform(ctx, math::vec3f(bbox.min), "bbx_min");
                warp_pass_program_->set_uniform(ctx, math::vec3f(bbox.max), "bbx_max");

                // pre passes
                for(unsigned layer = 0; layer != video3d_ressource->number_of_cameras(); ++layer)
                {
                    // configure fbo
                    warp_result_fbo_->clear_attachments();
                    warp_result_fbo_->attach_depth_stencil_buffer(warp_depth_result_, 0, layer);
                    warp_result_fbo_->attach_color_buffer(0, warp_color_result_, 0, layer);

                    // bind and clear fbo
                    ctx.render_context->set_frame_buffer(warp_result_fbo_);
                    ctx.render_context->clear_depth_stencil_buffer(warp_result_fbo_);
                    ctx.render_context->clear_color_buffer(warp_result_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
                    ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2ui(0, 0), warp_color_result_->dimensions()));

                    // set uniforms
                    warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).getTexSizeInvD(), "tex_size_inv");
                    warp_pass_program_->set_uniform(ctx, int(layer), "layer");

                    ctx.render_context->bind_texture(video3d_data.cv_xyz_[layer], linear_sampler_state_, 1);

                    ctx.render_context->bind_texture(video3d_data.cv_uv_[layer], linear_sampler_state_, 2);

                    warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).cv_min_d, "cv_min_d");
                    warp_pass_program_->set_uniform(ctx, video3d_ressource->calibration_file(layer).cv_max_d, "cv_max_d");

                    warp_pass_program_->use(ctx);
                    {
                        draw_video3dResource(pipe.get_context(), *video3d_ressource);
                    }
                    warp_pass_program_->unuse(ctx);

                    ctx.render_context->reset_framebuffer();
                }
            }

            // get material dependent shader
            std::shared_ptr<ShaderProgram> current_shader;

            MaterialShader* current_material = video_node->get_material()->get_shader();
            if(current_material)
            {
                auto shader_iterator = programs_.find(current_material);
                if(shader_iterator != programs_.end())
                {
                    current_shader = shader_iterator->second;
                }
                else
                {
                    auto smap = global_substitution_map_;
                    for(const auto& i : current_material->generate_substitution_map())
                        smap[i.first] = i.second;

                    current_shader = std::make_shared<ShaderProgram>();
                    current_shader->set_shaders(program_stages_, std::list<std::string>(), false, smap);
                    programs_[current_material] = current_shader;
                }
            }
            else
            {
                Logger::LOG_WARNING << "Video3DPass::render(): Cannot find material: " << video_node->get_material()->get_shader_name() << std::endl;
            }

            current_shader->use(ctx);

            bool write_depth = true;
            target.bind(ctx, write_depth);
            target.set_viewport(ctx);

            {
                // single texture only
                scm::gl::context_all_guard guard(ctx.render_context);

                ctx.render_context->set_depth_stencil_state(depth_stencil_state_warp_pass_);

                // second pass
                {
                    current_shader->apply_uniform(ctx, "gua_normal_matrix", gua::math::mat4f(normal_matrix));
                    current_shader->apply_uniform(ctx, "gua_model_matrix", gua::math::mat4f(model_matrix));

                    // needs to be multiplied with scene scaling
                    current_shader->set_uniform(ctx, 0.075f * scaling, "epsilon");
                    current_shader->set_uniform(ctx, int(video3d_ressource->number_of_cameras()), "numlayers");
                    current_shader->set_uniform(ctx, int(video3d_ressource->do_overwrite_normal()), "overwrite_normal");
                    current_shader->set_uniform(ctx, video3d_ressource->get_overwrite_normal(), "o_normal");

                    ctx.render_context->bind_texture(warp_color_result_, nearest_sampler_state_, 0);
                    current_shader->get_program()->uniform_sampler("quality_texture", 0);

                    ctx.render_context->bind_texture(warp_depth_result_, nearest_sampler_state_, 1);
                    current_shader->get_program()->uniform_sampler("depth_texture", 1);

                    ctx.render_context->bind_texture(video3d_data.color_tex_, linear_sampler_state_, 2);
                    current_shader->get_program()->uniform_sampler("video_color_texture", 2);

                    video_node->get_material()->apply_uniforms(ctx, current_shader.get(), view_id);

                    pipe.draw_quad();
                }
                current_shader->unuse(ctx);
            }

            target.unbind(ctx);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
