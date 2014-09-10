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
#include <gua/renderer/WindowBase.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <sstream>
#include <iostream>

namespace gua {

std::string subroutine_from_mode(WindowBase::TextureDisplayMode mode) {
  switch (mode) {
    case WindowBase::RED:
      return "get_red";
      break;
    case WindowBase::GREEN:
      return "get_green";
      break;
    case WindowBase::CYAN:
      return "get_cyan";
      break;
    case WindowBase::CHECKER_EVEN:
      return "get_checker_even";
      break;
    case WindowBase::CHECKER_ODD:
      return "get_checker_odd";
      break;
    default:
      return "get_full";
  }
}

////////////////////////////////////////////////////////////////////////////////

std::atomic_uint WindowBase::last_context_id_{ 0 };
std::mutex WindowBase::last_context_id_mutex_{};

////////////////////////////////////////////////////////////////////////////////

WindowBase::WindowBase(Configuration const& configuration)
    : config(configuration),
      fullscreen_shader_(),
      fullscreen_quad_(),
      depth_stencil_state_(),
      warpRR_(nullptr),
      warpGR_(nullptr),
      warpBR_(nullptr),
      warpRL_(nullptr),
      warpGL_(nullptr),
      warpBL_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

WindowBase::~WindowBase() {}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::open() {
  set_active(true);

  ctx_.render_device = scm::gl::render_device_ptr(new scm::gl::render_device());
  ctx_.render_context = ctx_.render_device->main_context();

  {
    std::lock_guard<std::mutex> lock(last_context_id_mutex_);
    ctx_.id = last_context_id_++;
  }

  ctx_.render_window = this;

  fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
      ctx_.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));

  depth_stencil_state_ = ctx_.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

  blend_state_ = ctx_.render_device->create_blend_state(true,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE);
  if (config.get_debug()) {
    ctx_.render_context->register_debug_callback(boost::make_shared<DebugOutput>());
  }
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::create_shader() {

  if (config.get_warp_matrix_red_right() == "" ||
      config.get_warp_matrix_green_right() == "" ||
      config.get_warp_matrix_blue_right() == "" ||
      config.get_warp_matrix_red_left() == "" ||
      config.get_warp_matrix_green_left() == "" ||
      config.get_warp_matrix_blue_left() == "") {

    fullscreen_shader_.create_from_sources(
      Resources::lookup_shader(Resources::shaders_display_shader_vert),
      Resources::lookup_shader(Resources::shaders_display_shader_frag));
  } else {
    warpRR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_red_right());

    warpGR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_green_right());

    warpBR_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_blue_right());

    warpRL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_red_left());

    warpGL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_green_left());

    warpBL_ = std::make_shared<WarpMatrix>(config.get_warp_matrix_blue_left());

    fullscreen_shader_.create_from_sources(
      Resources::lookup_shader(Resources::shaders_display_shader_vert),
      Resources::lookup_shader(Resources::shaders_display_shader_warped_frag)
    );
  }
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::start_frame() const {
  ctx_.render_context->clear_default_color_buffer(
      scm::gl::FRAMEBUFFER_BACK, scm::math::vec4f(0.f, 0.f, 0.f, 1.0f));

  ctx_.render_context->clear_default_depth_stencil_buffer();
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(std::shared_ptr<Texture2D> const& center_texture) {

  display(center_texture, config.get_left_resolution(),
          config.get_left_position(), WindowBase::FULL, true, true);

}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(std::shared_ptr<Texture2D> const& left_texture,
                     std::shared_ptr<Texture2D> const& right_texture) {

  switch (config.get_stereo_mode()) {
    case StereoMode::MONO:
      display(left_texture);
      break;
    case StereoMode::SIDE_BY_SIDE:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), WindowBase::FULL, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), WindowBase::FULL, false, true);
      break;
    case StereoMode::ANAGLYPH_RED_CYAN:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), WindowBase::RED, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), WindowBase::CYAN, false, false);
      break;
    case StereoMode::ANAGLYPH_RED_GREEN:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), WindowBase::RED, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), WindowBase::GREEN, false, false);
      break;
    case StereoMode::CHECKERBOARD:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), WindowBase::CHECKER_EVEN, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), WindowBase::CHECKER_ODD, false, true);
      break;
  }

}

////////////////////////////////////////////////////////////////////////////////

RenderContext* WindowBase::get_context() { return &ctx_; }

////////////////////////////////////////////////////////////////////////////////

void WindowBase::display(std::shared_ptr<Texture2D> const& texture,
                     math::vec2ui const& size,
                     math::vec2ui const& position,
                     TextureDisplayMode mode,
                     bool is_left,
                     bool clear) {

  fullscreen_shader_.use(ctx_);
  fullscreen_shader_.set_uniform(ctx_, texture, "sampler");

  if (is_left) {
    if (warpRL_) fullscreen_shader_.set_uniform(ctx_, std::dynamic_pointer_cast<Texture2D>(warpRL_), "warpR");
    if (warpGL_) fullscreen_shader_.set_uniform(ctx_, std::dynamic_pointer_cast<Texture2D>(warpGL_), "warpG");
    if (warpBL_) fullscreen_shader_.set_uniform(ctx_, std::dynamic_pointer_cast<Texture2D>(warpBL_), "warpB");
  } else {
    if (warpRR_) fullscreen_shader_.set_uniform(ctx_, std::dynamic_pointer_cast<Texture2D>(warpRR_), "warpR");
    if (warpGR_) fullscreen_shader_.set_uniform(ctx_, std::dynamic_pointer_cast<Texture2D>(warpGR_), "warpG");
    if (warpBR_) fullscreen_shader_.set_uniform(ctx_, std::dynamic_pointer_cast<Texture2D>(warpBR_), "warpB");
  }

  std::string subroutine = subroutine_from_mode(mode);

  fullscreen_shader_.set_subroutine(
      ctx_, scm::gl::STAGE_FRAGMENT_SHADER, "get_color", subroutine);

  ctx_.render_context->set_viewport(scm::gl::viewport(position, size));
  ctx_.render_context->set_depth_stencil_state(depth_stencil_state_);

  if (!clear) {
    ctx_.render_context->set_blend_state(blend_state_);
  }

  fullscreen_quad_->draw(ctx_.render_context);

  ctx_.render_context->reset_state_objects();
  fullscreen_shader_.unuse(ctx_);
}

////////////////////////////////////////////////////////////////////////////////

void WindowBase::DebugOutput::operator()(scm::gl::debug_source source,
                                     scm::gl::debug_type type,
                                     scm::gl::debug_severity severity,
                                     const std::string& message) const {

  Logger::LOG_MESSAGE << "[Source: " << scm::gl::debug_source_string(source)
                      << ", type: " << scm::gl::debug_type_string(type)
                      << ", severity: " << scm::gl::debug_severity_string(severity)
                      << "]: " << message << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

}
