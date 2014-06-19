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
#include <gua/renderer/PostFXPass.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/databases.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils.hpp>
#include <gua/renderer/DisplayData.hpp>

#include <gua/renderer/LightingPass.hpp>

#define LUMINANCE_MAP_SIZE 512

namespace gua {

////////////////////////////////////////////////////////////////////////////////

PostFXPass::PostFXPass(Pipeline* pipeline):
    Pass(pipeline),
    postfx_shaders_(),
    ping_buffer_(nullptr),
    pong_buffer_(nullptr),
    fps_text_renderer_(nullptr),
    preview_text_renderer_(nullptr),
    god_ray_shader_(nullptr),
    fullscreen_texture_shader_(nullptr),
    glow_shader_(nullptr),
    luminance_shader_(nullptr),
    luminance_buffer_(nullptr),
    last_frame_luminance_(1.f) {

    ////////////////////////////////////////////////////////////////////////////

    postfx_shaders_.push_back(new ShaderProgram());

    postfx_shaders_.back()->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_stage_00_frag)
    );

    postfx_shaders_.push_back(new ShaderProgram());
    postfx_shaders_.back()->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_stage_01_frag)
    );

    postfx_shaders_.push_back(new ShaderProgram());
    postfx_shaders_.back()->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_stage_02_frag)
    );

    postfx_shaders_.push_back(new ShaderProgram());
    postfx_shaders_.back()->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_stage_03_frag)
    );

    ////////////////////////////////////////////////////////////////////////////

    // godray shader -----------------------------------------------------------
    god_ray_shader_ = new ShaderProgram();
    god_ray_shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_godrays_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_godrays_frag)
    );

    // fullscreen texture shader -----------------------------------------------
    fullscreen_texture_shader_ = new ShaderProgram();
    fullscreen_texture_shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_frag)
    );

    // luminance shader --------------------------------------------------------
    luminance_shader_ = new ShaderProgram();
    luminance_shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_common_fullscreen_quad_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_luminance_frag)
    );

    // glow shader -------------------------------------------------------------
    glow_shader_ = new ShaderProgram();
    glow_shader_->create_from_sources(
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_glow_vert),
      Resources::lookup_shader(Resources::shaders_uber_shaders_postfx_glow_frag)
    );

}

////////////////////////////////////////////////////////////////////////////////

PostFXPass::~PostFXPass() {

  for (auto p: postfx_shaders_) {
    delete p;
  }

  for (auto p: godray_buffers_) {
    delete p;
  }

  for (auto p: glow_buffers_) {
    delete p;
  }

  if (ping_buffer_) {
    delete ping_buffer_;
  }

  if (pong_buffer_) {
    delete pong_buffer_;
  }

  if (fps_text_renderer_) {
    delete fps_text_renderer_;
  }

  if (preview_text_renderer_) {
    delete preview_text_renderer_;
  }


  if (god_ray_shader_) {
    delete god_ray_shader_;
  }

  if (fullscreen_texture_shader_) {
    delete fullscreen_texture_shader_;
  }

  if (glow_shader_) {
    delete glow_shader_;
  }

  if (luminance_shader_) {
    delete luminance_shader_;
  }

  if (luminance_buffer_) {
    delete luminance_buffer_;
  }
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::create(RenderContext const& ctx, std::vector<std::pair<BufferComponent,
    scm::gl::sampler_state_desc>> const& layers) {
  Pass::create(ctx, layers);

  for (auto p: godray_buffers_) {
    p->remove_buffers(ctx);
    delete p;
  }

  godray_buffers_.clear();

  for (auto p: glow_buffers_) {
    p->remove_buffers(ctx);
    delete p;
  }

  glow_buffers_.clear();

  if (ping_buffer_) {
    ping_buffer_->remove_buffers(ctx);
    delete ping_buffer_;
  }

  if (pong_buffer_) {
    pong_buffer_->remove_buffers(ctx);
    delete pong_buffer_;
  }

  if (luminance_buffer_) {
    luminance_buffer_->remove_buffers(ctx);
    delete luminance_buffer_;
  }

  ping_buffer_ = new StereoBuffer(ctx, pipeline_->config, layers);
  pong_buffer_ = new StereoBuffer(ctx, pipeline_->config, layers);

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
                                    scm::gl::WRAP_CLAMP_TO_EDGE,
                                    scm::gl::WRAP_CLAMP_TO_EDGE);

  std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc>> layer_3f_desc;
  layer_3f_desc.push_back(std::make_pair(BufferComponent::F3, state));

  godray_buffers_.push_back(new GBuffer(layer_3f_desc,
                                pipeline_->config.get_left_resolution()[0]/2,
                                pipeline_->config.get_left_resolution()[1]/2));
  godray_buffers_.push_back(new GBuffer(layer_3f_desc,
                                pipeline_->config.get_left_resolution()[0]/2,
                                pipeline_->config.get_left_resolution()[1]/2));
  godray_buffers_.push_back(new GBuffer(layer_3f_desc,
                                pipeline_->config.get_left_resolution()[0]/2,
                                pipeline_->config.get_left_resolution()[1]/2));

  for (auto buffer: godray_buffers_) {
      buffer->create(ctx);
  }

  glow_buffers_.push_back(new GBuffer(layer_3f_desc,
                                      pipeline_->config.get_left_resolution()[0]/2,
                                      pipeline_->config.get_left_resolution()[1]/2));
  glow_buffers_.push_back(new GBuffer(layer_3f_desc,
                                      pipeline_->config.get_left_resolution()[0]/2,
                                      pipeline_->config.get_left_resolution()[1]/2));

  for (auto buffer: glow_buffers_) {
      buffer->create(ctx);
  }

  std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc>> layer_1f_desc;
  layer_1f_desc.push_back(std::make_pair(BufferComponent::F1, state));

  // todo: luminance buffer resolution configurable
  luminance_buffer_ = new GBuffer(layer_1f_desc, LUMINANCE_MAP_SIZE,
                                  LUMINANCE_MAP_SIZE,
                                  std::log(LUMINANCE_MAP_SIZE) / std::log(2));
  luminance_buffer_->create_UGLY(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::cleanup(RenderContext const& ctx) {
  if (ping_buffer_) ping_buffer_->remove_buffers(ctx);
  if (pong_buffer_) pong_buffer_->remove_buffers(ctx);

  for (auto buffer : godray_buffers_)
    if (buffer) buffer->remove_buffers(ctx);

  for (auto buffer : glow_buffers_)
    if (buffer) buffer->remove_buffers(ctx);

  if (luminance_buffer_) luminance_buffer_->remove_buffers(ctx);

  Pass::cleanup(ctx);
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::init_resources(RenderContext const& ctx) {
  if (!initialized_) {
    if (!depth_stencil_state_)
        depth_stencil_state_ = ctx.render_device->
            create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

    if (!fullscreen_quad_)
        fullscreen_quad_ = scm::gl::quad_geometry_ptr(
            new scm::gl::quad_geometry(ctx.render_device,
                                                       math::vec2(-1.f, -1.f),
                                                       math::vec2( 1.f,  1.f)));
    if (!blend_add_)
        blend_add_ = ctx.render_device->
                         create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE,
                                                  scm::gl::FUNC_ONE, scm::gl::FUNC_ONE);
    initialized_ = true;
  }
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::render_scene(Camera const& camera,
                              SceneGraph const&,
                              RenderContext const& ctx,
                              std::size_t viewid) {
    init_resources(ctx);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_);

    render_fxaa(ctx);
    render_fog(ctx);
    render_vignette(ctx);
    render_ssao(ctx);


    for (int i(0); i<gbuffer_->get_eye_buffers().size(); ++i) {
        CameraMode eye(CameraMode::CENTER);

        if (gbuffer_->get_eye_buffers().size() > 1 && i == 0) eye = CameraMode::LEFT;
        if (gbuffer_->get_eye_buffers().size() > 1 && i == 1) eye = CameraMode::RIGHT;

        postfx_shaders_[0]->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
        postfx_shaders_[0]->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");
        postfx_shaders_[3]->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_width");
        postfx_shaders_[3]->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_height");


        bool any_godrays(false);
        // ---------------------------------------------------------------------

        any_godrays = render_godrays(camera, pipeline_->get_current_scene(eye), eye, ctx);
        render_glow(eye, ctx);

        auto input_tex(inputs_[Pipeline::compositing]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
        auto ping_tex(ping_buffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
        auto pong_tex(pong_buffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
        auto normal_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
        auto depth_tex(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_depth_buffer());

        Pass::set_camera_matrices(*postfx_shaders_[0], camera, pipeline_->get_current_scene(eye), eye, ctx);
        pipeline_->camera_block_->update(ctx.render_context, pipeline_->get_current_scene(eye).frustum);
        ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_->block().block_buffer(), 0);

        // ---------------------------------------------------------------------
        auto current_target(ping_buffer_);
        auto current_input(input_tex);

        auto swap_target = [&]() {
            if (current_target == ping_buffer_) {
                current_target = pong_buffer_;
                current_input = ping_tex;
            } else {
                current_target = ping_buffer_;
                current_input = pong_tex;
            }

            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->set_viewport(ctx);
        };

        current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->set_viewport(ctx);

        if (pipeline_->config.enable_fog() || pipeline_->config.enable_ssao() || any_godrays) {
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->bind(ctx);
            postfx_shaders_[0]->set_uniform(ctx, current_input, "gua_color_gbuffer_in");
            postfx_shaders_[0]->set_uniform(ctx, normal_tex, "gua_normal_gbuffer_in");
            postfx_shaders_[0]->set_uniform(ctx, depth_tex, "gua_depth_gbuffer_in");
            postfx_shaders_[0]->use(ctx);
            fullscreen_quad_->draw(ctx.render_context);
            postfx_shaders_[0]->unuse(ctx);
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->unbind(ctx);

            swap_target();
        }

        if (pipeline_->config.enable_bloom()) {
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->bind(ctx);
            postfx_shaders_[1]->set_uniform(ctx, current_input, "gua_color_gbuffer_in");
            postfx_shaders_[1]->use(ctx);
            fullscreen_quad_->draw(ctx.render_context);
            postfx_shaders_[1]->unuse(ctx);
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->unbind(ctx);

            swap_target();
        }

        if (pipeline_->config.enable_hdr()) {

            render_hdr(ctx, current_input);
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->set_viewport(ctx);

            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->bind(ctx);
            postfx_shaders_[2]->set_uniform(ctx, current_input, "gua_color_gbuffer_in");
            postfx_shaders_[2]->use(ctx);
            fullscreen_quad_->draw(ctx.render_context);
            postfx_shaders_[2]->unuse(ctx);
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->unbind(ctx);

            swap_target();
        }

        if (pipeline_->config.enable_fxaa() || pipeline_->config.enable_vignette()) {
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->bind(ctx);
            postfx_shaders_[3]->set_uniform(ctx, current_input, "gua_color_gbuffer_in");
            postfx_shaders_[3]->use(ctx);
            fullscreen_quad_->draw(ctx.render_context);
            postfx_shaders_[3]->unuse(ctx);
            current_target->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->unbind(ctx);

            swap_target();
        }


        gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->bind(ctx);
        fullscreen_texture_shader_->set_uniform(ctx, current_input, "gua_in_texture");
        fullscreen_texture_shader_->use(ctx);
        fullscreen_quad_->draw(ctx.render_context);
        fullscreen_texture_shader_->unuse(ctx);
        gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->unbind(ctx);



        render_previews(eye, ctx);
    }

    ctx.render_context->reset_state_objects();

}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::render_fog(RenderContext const& ctx) {

    postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.enable_fog(), "gua_enable_fog");

    if (pipeline_->config.enable_fog()) {
        postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.fog_start(), "gua_fog_start");
        postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.fog_end(), "gua_fog_end");
        postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.fog_texture().empty(), "gua_fog_is_color");

        if (pipeline_->config.fog_texture().empty())
            postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.fog_color(), "gua_fog_color");
        else
            postfx_shaders_[0]->set_uniform(ctx, TextureDatabase::instance()->lookup(pipeline_->config.fog_texture()), "gua_fog_texture");
    }
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::render_vignette(RenderContext const& ctx) {
    postfx_shaders_[3]->set_uniform(ctx, pipeline_->config.enable_vignette(),
                                    "gua_enable_vignette");

    if (pipeline_->config.enable_vignette()) {
        postfx_shaders_[3]->set_uniform(ctx,
                pipeline_->config.vignette_coverage(), "gua_vignette_coverage");
        postfx_shaders_[3]->set_uniform(ctx,
                pipeline_->config.vignette_softness(), "gua_vignette_softness");
        postfx_shaders_[3]->set_uniform(ctx,
                pipeline_->config.vignette_color(), "gua_vignette_color");
    }
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::render_fxaa(RenderContext const& ctx) {
    postfx_shaders_[3]->set_uniform(ctx, pipeline_->config.enable_fxaa(),
                                    "gua_enable_fxaa");
}

bool PostFXPass::render_godrays(Camera const& camera,
                                SerializedScene const& scene,
                                CameraMode eye, RenderContext const& ctx) {

    bool any_godrays(false);
    for (auto const& light: scene.point_lights_) {
        if (light->data.get_enable_godrays()) {
            any_godrays = true;
            break;
        }
    }

    if (!any_godrays) {
        for (auto const& light: scene.spot_lights_) {
            if (light->data.get_enable_godrays()) {
                any_godrays = true;
                break;
            }
        }
    }

    if (!any_godrays) {
        for (auto const& light: scene.sun_lights_) {
            if (light->data.get_enable_godrays()) {
                any_godrays = true;
                break;
            }
        }
    }

    postfx_shaders_[0]->set_uniform(ctx, any_godrays, "gua_enable_godrays");

    if (any_godrays) {
        Pass::set_camera_matrices(*god_ray_shader_, camera, scene, eye, ctx);
        pipeline_->camera_block_->update(ctx.render_context, scene.frustum);
        ctx.render_context->bind_uniform_buffer(pipeline_->camera_block_->block().block_buffer(), 0);

        auto depth_buffer(inputs_[Pipeline::geometry]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_depth_buffer());
        god_ray_shader_->set_uniform(ctx, 1.0f * godray_buffers_[0]->width() / godray_buffers_[0]->height(), "gua_aspect_ratio");
        ctx.render_context->set_viewport(scm::gl::viewport(
                             math::vec2(0.0f,0.0f), math::vec2(float(godray_buffers_[0]->width()), float(godray_buffers_[0]->height()))));

        godray_buffers_[2]->clear_color_buffers(ctx);

        auto render = [&](){
            // first pass ----------------------------------------------------------
            god_ray_shader_->use(ctx);
            godray_buffers_[0]->bind(ctx);
            god_ray_shader_->set_uniform(ctx, depth_buffer, "gua_ray_texture");
            god_ray_shader_->set_uniform(ctx, 1.0f, "gua_filter_length");
            god_ray_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_clamped");
            fullscreen_quad_->draw(ctx.render_context);
            godray_buffers_[0]->unbind(ctx);

            // second pass ---------------------------------------------------------
            godray_buffers_[1]->bind(ctx);
            god_ray_shader_->set_uniform(ctx, godray_buffers_[0]->get_color_buffers(TYPE_FLOAT)[0], "gua_ray_texture");
            god_ray_shader_->set_uniform(ctx, 2.f, "gua_filter_length");
            god_ray_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_smooth");
            fullscreen_quad_->draw(ctx.render_context);
            godray_buffers_[1]->unbind(ctx);

            // third pass ----------------------------------------------------------
            godray_buffers_[0]->bind(ctx);
            god_ray_shader_->set_uniform(ctx, godray_buffers_[1]->get_color_buffers(TYPE_FLOAT)[0], "gua_ray_texture");
            god_ray_shader_->set_uniform(ctx, 6.f, "gua_filter_length");
            fullscreen_quad_->draw(ctx.render_context);
            godray_buffers_[0]->unbind(ctx);

            // accumulation pass ----------------------------------------------------------
            ctx.render_context->set_blend_state(blend_add_);
            godray_buffers_[2]->bind(ctx);
            fullscreen_texture_shader_->use(ctx);
            fullscreen_texture_shader_->set_uniform(ctx, godray_buffers_[0]->get_color_buffers(TYPE_FLOAT)[0], "gua_in_texture");
            fullscreen_quad_->draw(ctx.render_context);
            godray_buffers_[2]->unbind(ctx);
            ctx.render_context->reset_state_objects();
        };

        god_ray_shader_->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_position",
                            "gua_calculate_by_position");

        for (auto const& light: scene.point_lights_) {
            if (light->data.get_enable_godrays()) {
                god_ray_shader_->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
                god_ray_shader_->set_uniform(ctx, math::vec3(light->get_cached_world_transform().column(3)[0],
                                                             light->get_cached_world_transform().column(3)[1],
                                                             light->get_cached_world_transform().column(3)[2]), "gua_light_position_direction");
                render();
            }
        }

        for (auto const& light: scene.spot_lights_) {
            if (light->data.get_enable_godrays()) {
                god_ray_shader_->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
                god_ray_shader_->set_uniform(ctx, math::vec3(light->get_cached_world_transform().column(3)[0],
                                                             light->get_cached_world_transform().column(3)[1],
                                                             light->get_cached_world_transform().column(3)[2]), "gua_light_position_direction");
                render();
            }
        }

        god_ray_shader_->set_subroutine(ctx,
                            scm::gl::STAGE_VERTEX_SHADER,
                            "compute_position",
                            "gua_calculate_by_direction");

        for (auto const& light: scene.sun_lights_) {
            if (light->data.get_enable_godrays()) {
                god_ray_shader_->set_uniform(ctx, light->data.get_color().vec3(), "gua_light_color");
                math::vec3 direction(0, 0, 1);
                direction = light->get_cached_world_transform() * direction;
                god_ray_shader_->set_uniform(ctx, direction, "gua_light_position_direction");
                render();
            }
        }

        postfx_shaders_[0]->set_uniform(ctx, godray_buffers_[2]->get_color_buffers(TYPE_FLOAT)[0], "gua_god_rays_texture");

        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::render_glow(CameraMode eye, RenderContext const& ctx) {
    postfx_shaders_[1]->set_uniform(ctx, pipeline_->config.enable_bloom(), "gua_enable_glow");

    if (pipeline_->config.enable_bloom()) {

        postfx_shaders_[1]->set_uniform(ctx, pipeline_->config.bloom_intensity(), "gua_glow_intensity");

        glow_shader_->set_uniform(ctx, 1.0f * glow_buffers_[0]->width() / glow_buffers_[0]->height(), "gua_aspect_ratio");
        glow_shader_->set_uniform(ctx, pipeline_->config.bloom_radius(), "gua_glow_radius");
        glow_shader_->set_uniform(ctx, pipeline_->config.bloom_threshold(), "gua_glow_threshold");


        auto color_buffer(inputs_[Pipeline::compositing]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(TYPE_FLOAT)[0]);
        ctx.render_context->set_viewport(scm::gl::viewport(
                math::vec2(0,0), math::vec2(float(glow_buffers_[0]->width()),
                                            float(glow_buffers_[0]->height()))));


        // threshold pass ----------------------------------------------------------
        glow_shader_->use(ctx);
        glow_buffers_[0]->bind(ctx);
        glow_shader_->set_uniform(ctx, 0.f, "gua_texel_size");
        glow_shader_->set_uniform(ctx, math::vec2(0, 0), "gua_blur_direction");
        glow_shader_->set_uniform(ctx, color_buffer, "gua_glow_texture");
        glow_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_threshold");
        fullscreen_quad_->draw(ctx.render_context);
        glow_buffers_[0]->unbind(ctx);

        // first x-blur pass ---------------------------------------------------------
        glow_buffers_[1]->bind(ctx);
        glow_shader_->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_size");
        glow_shader_->set_uniform(ctx, math::vec2(1, 0), "gua_blur_direction");
        glow_shader_->set_uniform(ctx, glow_buffers_[0]->get_color_buffers(TYPE_FLOAT)[0], "gua_glow_texture");
        glow_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_smooth");
        fullscreen_quad_->draw(ctx.render_context);
        glow_buffers_[1]->unbind(ctx);

        // first y-blur pass ---------------------------------------------------------
        glow_buffers_[0]->bind(ctx);
        glow_shader_->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_size");
        glow_shader_->set_uniform(ctx, math::vec2(0, 1), "gua_blur_direction");
        glow_shader_->set_uniform(ctx, glow_buffers_[1]->get_color_buffers(TYPE_FLOAT)[0], "gua_glow_texture");
        glow_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_smooth");
        fullscreen_quad_->draw(ctx.render_context);
        glow_buffers_[0]->unbind(ctx);

        // second x-blur pass ---------------------------------------------------------
        glow_buffers_[1]->bind(ctx);
        glow_shader_->set_uniform(ctx, pipeline_->config.bloom_radius()/7.0, "gua_glow_radius");
        glow_shader_->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), "gua_texel_size");
        glow_shader_->set_uniform(ctx, math::vec2(1, 0), "gua_blur_direction");
        glow_shader_->set_uniform(ctx, glow_buffers_[0]->get_color_buffers(TYPE_FLOAT)[0], "gua_glow_texture");
        glow_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_smooth");
        fullscreen_quad_->draw(ctx.render_context);
        glow_buffers_[1]->unbind(ctx);

        // second y-blur pass ---------------------------------------------------------
        glow_buffers_[0]->bind(ctx);
        glow_shader_->set_uniform(ctx, 1.f/gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height(), "gua_texel_size");
        glow_shader_->set_uniform(ctx, math::vec2(0, 1), "gua_blur_direction");
        glow_shader_->set_uniform(ctx, glow_buffers_[1]->get_color_buffers(TYPE_FLOAT)[0], "gua_glow_texture");
        glow_shader_->set_subroutine(ctx, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", "get_color_smooth");
        fullscreen_quad_->draw(ctx.render_context);
        glow_buffers_[0]->unbind(ctx);

        postfx_shaders_[1]->set_uniform(ctx, glow_buffers_[0]->get_color_buffers(TYPE_FLOAT)[0], "gua_glow_texture");
    }
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::render_ssao(RenderContext const& ctx) {

    postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.enable_ssao(), "gua_enable_ssao");

    if (pipeline_->config.enable_ssao()) {
        postfx_shaders_[0]->set_uniform(ctx, noise_texture_.get_handle(ctx),
            "gua_noise_tex");
        postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.ssao_falloff(),
            "gua_ssao_falloff");
        postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.ssao_intensity(),
            "gua_ssao_intensity");
        postfx_shaders_[0]->set_uniform(ctx, pipeline_->config.ssao_radius(),
            "gua_ssao_radius");
    }
}

////////////////////////////////////////////////////////////////////////////////

void PostFXPass::
render_hdr(RenderContext const& ctx, std::shared_ptr<Texture2D> const& texture) {

    ctx.render_context->set_viewport(scm::gl::viewport(
            math::vec2(0,0), math::vec2(float(luminance_buffer_->width()),
                                        float(luminance_buffer_->height()))));

    luminance_buffer_->bind(ctx);
    luminance_shader_->use(ctx);
    luminance_shader_->set_uniform(ctx, texture, "gua_in_texture");
    fullscreen_quad_->draw(ctx.render_context);
    luminance_buffer_->unbind(ctx);

    auto luminance_texture(luminance_buffer_->get_color_buffers(TYPE_FLOAT)[0]);
    luminance_texture->generate_mipmaps(ctx);

    float luminance[3] = {0.f, 0.f, 0.f};
    ctx.render_context->retrieve_texture_data(luminance_texture->get_buffer(ctx), std::log(LUMINANCE_MAP_SIZE) / std::log(2) - 1, luminance);

    if(luminance[0] > 0.f) {
        float adaption(0.4f);
        last_frame_luminance_ = last_frame_luminance_ + (luminance[0] - last_frame_luminance_) * (1.f - std::exp(-0.016f/adaption));
    }

    postfx_shaders_[2]->set_uniform(ctx, last_frame_luminance_, "gua_luminance");
    postfx_shaders_[2]->set_uniform(ctx, pipeline_->config.get_hdr_key(), "gua_hdr_key");
}

void PostFXPass::render_previews(CameraMode eye, RenderContext const& ctx) {

    #if GUA_COMPILER == GUA_COMPILER_MSVC
        const std::string font("Consola.ttf");
    #else
        const std::string font("FreeSans.ttf");
    #endif

    auto fbo(gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]);
        fbo->bind(ctx);

    if (pipeline_->config.enable_fps_display()) {
        if (!fps_text_renderer_) {
          fps_text_renderer_ = new TextRenderer(ctx, 16, font);
        }

        DisplayData dd;

        std::string text("Application FPS: "
                            + string_utils::to_string(pipeline_->get_application_fps())
                            + "\nRendering FPS: "
                            + string_utils::to_string(pipeline_->get_rendering_fps())
                            + ((dd.get_physics_fps() == 0.f) ? "" : "\nPhysics FPS: "
                            + string_utils::to_string(dd.get_physics_fps())));

        fps_text_renderer_->render_outlined(ctx, *gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0], text, math::vec2i(40, gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height() - 40),
                                            utils::Color3f(1, 1, 1),utils::Color3f(0, 0, 0));
    }


    if (pipeline_->config.enable_preview_display()) {
        if (!preview_text_renderer_) {
            preview_text_renderer_ = new TextRenderer(ctx, 12, font);
        }

        std::vector<std::pair<std::string, std::shared_ptr<Texture2D>>> previews;

        for (unsigned input(0); input < inputs_.size(); ++input) {

            auto depth(inputs_[input]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_depth_buffer());
            if (depth) {
                std::string name("Stage " + string_utils::to_string(input)
                                 + ", Depth Layer");
                previews.push_back(std::make_pair(name, depth));
            }

            for (int type(TYPE_INTEGER); type < TYPE_DEPTH; ++type) {
                auto layers(inputs_[input]->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->get_color_buffers(BufferComponentType(type)));
                for (unsigned layer(0); layer < layers.size(); ++layer) {
                    std::string name("Stage " + string_utils::to_string(input)
                                     + ", " + enums::buffer_component_type_to_string(BufferComponentType(type))
                                     + "-Layer " + string_utils::to_string(layer));
                    previews.push_back(std::make_pair(name, layers[layer]));
                }
            }
        }

        for(unsigned i(0); i<previews.size(); ++i) {
            math::vec2 position(i * gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width()/previews.size(), 0);
            float scale = 1.0f/previews.size();

            fullscreen_texture_shader_->use(ctx);

            fullscreen_texture_shader_->set_uniform(ctx, previews[i].second, "gua_in_texture", 0);

            ctx.render_context->set_viewport(scm::gl::viewport(position,
                                                  math::vec2(gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width() * scale, gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height() * scale)));

            fullscreen_quad_->draw(ctx.render_context);

            ctx.render_context->reset_state_objects();
            fullscreen_texture_shader_->unuse(ctx);

            ctx.render_context->set_viewport(scm::gl::viewport(math::vec2(0,0),
                                                  math::vec2(gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->width(), gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0]->height())));
            preview_text_renderer_->render_outlined(ctx, *gbuffer_->get_eye_buffers()[eye == CameraMode::RIGHT ? 1 : 0], previews[i].first, math::vec2i(position[0] + 10, position[1] + 10),
                                                    utils::Color3f(1, 1, 1),utils::Color3f(0, 0, 0));
        }



    }

    fbo->unbind(ctx);

}

/* virtual */ LayerMapping const* PostFXPass::get_gbuffer_mapping() const {
    throw std::runtime_error("no gbuffer mapping available for postfx pass");
}

void PostFXPass::print_shaders(std::string const& directory,
                               std::string const& name) const  {
    postfx_shaders_[0]->save_to_file(directory, name + "/stage_01");
    postfx_shaders_[1]->save_to_file(directory, name + "/stage_02");
    postfx_shaders_[2]->save_to_file(directory, name + "/stage_03");
    postfx_shaders_[3]->save_to_file(directory, name + "/stage_04");

    god_ray_shader_->save_to_file(directory, name + "/god_ray");
    fullscreen_texture_shader_->save_to_file(directory,
                                              name + "/fullscreen_texture");
    glow_shader_->save_to_file(directory, name + "/glow");
    luminance_shader_->save_to_file(directory, name + "/luminance");
}

////////////////////////////////////////////////////////////////////////////////

bool PostFXPass::pre_compile_shaders(RenderContext const& ctx) {

  bool success(true);

  for (auto shader: postfx_shaders_) {
    if (shader) success &= shader->upload_to(ctx);
  }

  if (god_ray_shader_)            success &= god_ray_shader_->upload_to(ctx);
  if (fullscreen_texture_shader_) success &= fullscreen_texture_shader_->upload_to(ctx);
  if (glow_shader_)               success &= glow_shader_->upload_to(ctx);
  if (luminance_shader_)          success &= luminance_shader_->upload_to(ctx);

  return success;
}

}
