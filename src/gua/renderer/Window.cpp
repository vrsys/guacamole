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
#include <gua/renderer/Window.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/StereoBuffer.hpp>
#include <gua/databases.hpp>
#include <gua/utils.hpp>

// external headers
#include <sstream>
#include <iostream>

namespace gua {

std::string subroutine_from_mode(Window::TextureDisplayMode mode) {
  switch (mode) {
    case Window::RED:
      return "get_red";
      break;
    case Window::GREEN:
      return "get_green";
      break;
    case Window::CYAN:
      return "get_cyan";
      break;
    case Window::CHECKER_EVEN:
      return "get_checker_even";
      break;
    case Window::CHECKER_ODD:
      return "get_checker_odd";
      break;
    default:
      return "get_full";
  }
}

////////////////////////////////////////////////////////////////////////////////

unsigned Window::last_context_id_ = 0;

////////////////////////////////////////////////////////////////////////////////

Window::Window(Configuration const& configuration)
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

Window::~Window() {

  close();
}

////////////////////////////////////////////////////////////////////////////////

void Window::open() {

  if (ctx_.window) {
    ctx_.window->hide();
  }

  ctx_.context.reset();
  ctx_.display.reset();
  ctx_.window.reset();

  scm::gl::wm::surface::format_desc window_format(
      scm::gl::FORMAT_RGBA_8, scm::gl::FORMAT_D24_S8, true, false);

#if GUA_COMPILER == GUA_COMPILER_MSVC
  scm::gl::wm::context::attribute_desc context_attribs(
      4, 3, false, false, false);
#else
  scm::gl::wm::context::attribute_desc context_attribs(
      4, 2, false, false, false);
#endif

  ctx_.display =
      scm::gl::wm::display_ptr(new scm::gl::wm::display(config.get_display_name()));

  ctx_.window = scm::gl::wm::window_ptr(new scm::gl::wm::window(
      ctx_.display,
      0,
      config.get_title(),
      math::vec2i(0, 0),
      math::vec2ui(config.get_size().x, config.get_size().y),
      window_format));

  ctx_.context = scm::gl::wm::context_ptr(
      new scm::gl::wm::context(ctx_.window, context_attribs));

  ctx_.window->show();

  set_active(true);

  ctx_.width = config.get_size().x;
  ctx_.height = config.get_size().y;
  ctx_.render_device = scm::gl::render_device_ptr(new scm::gl::render_device());
  ctx_.render_context = ctx_.render_device->main_context();
  ctx_.id = last_context_id_++;




  fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
      ctx_.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));

  depth_stencil_state_ = ctx_.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

  blend_state_ = ctx_.render_device->create_blend_state(true,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE);
}

////////////////////////////////////////////////////////////////////////////////

bool Window::get_is_open() const { return ctx_.window != nullptr; }

////////////////////////////////////////////////////////////////////////////////

void Window::create_shader() {

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

void Window::close() {
  if (get_is_open()) {

    ctx_.window->hide();
  }
}

////////////////////////////////////////////////////////////////////////////////

void Window::set_active(bool active) const {

  ctx_.context->make_current(ctx_.window, active);
}

////////////////////////////////////////////////////////////////////////////////

void Window::start_frame() const {
  ctx_.render_context->clear_default_color_buffer(
      scm::gl::FRAMEBUFFER_BACK, scm::math::vec4f(0.f, 0.f, 0.f, 1.0f));

  ctx_.render_context->clear_default_depth_stencil_buffer();
}

////////////////////////////////////////////////////////////////////////////////

void Window::finish_frame() const {

  set_active(true);
  ctx_.window->swap_buffers(config.get_enable_vsync());
}

////////////////////////////////////////////////////////////////////////////////

void Window::display(std::shared_ptr<Texture2D> const& center_texture) {

  display(center_texture, config.get_left_resolution(),
          config.get_left_position(), Window::FULL, true, true);

}

////////////////////////////////////////////////////////////////////////////////

void Window::display(std::shared_ptr<Texture2D> const& left_texture,
                     std::shared_ptr<Texture2D> const& right_texture) {

  switch (config.get_stereo_mode()) {
    case StereoMode::MONO:
      display(left_texture);
      break;
    case StereoMode::SIDE_BY_SIDE:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), Window::FULL, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), Window::FULL, false, true);
      break;
    case StereoMode::ANAGLYPH_RED_CYAN:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), Window::RED, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), Window::CYAN, false, false);
      break;
    case StereoMode::ANAGLYPH_RED_GREEN:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), Window::RED, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), Window::GREEN, false, false);
      break;
    case StereoMode::CHECKERBOARD:
      display(left_texture, config.get_left_resolution(),
              config.get_left_position(), Window::CHECKER_EVEN, true, true);
      display(right_texture, config.get_right_resolution(),
              config.get_right_position(), Window::CHECKER_ODD, false, true);
      break;
  }

}

////////////////////////////////////////////////////////////////////////////////

RenderContext* Window::get_context() { return &ctx_; }

////////////////////////////////////////////////////////////////////////////////

void Window::display(std::shared_ptr<Texture2D> const& texture,
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

  if (!clear)
    ctx_.render_context->set_blend_state(blend_state_);

  fullscreen_quad_->draw(ctx_.render_context);

  ctx_.render_context->reset_state_objects();
  fullscreen_shader_.unuse(ctx_);
}

////////////////////////////////////////////////////////////////////////////////


}
