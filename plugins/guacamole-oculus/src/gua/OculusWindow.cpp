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
#include <gua/OculusWindow.hpp>

// external headers
#include <iostream>

namespace gua {

OculusWindow::OculusWindow(std::string const& display):
  Window(),
  distortion_(4) {

  config.set_size(math::vec2ui(1280, 800));
  config.set_title("guacamole");
  config.set_display_name(display);
  config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);
  config.set_left_resolution(math::vec2ui(1280/2, 800));
  config.set_left_position(math::vec2ui(0, 0));
  config.set_right_resolution(math::vec2ui(1280/2, 800));
  config.set_right_position(math::vec2ui(1280/2, 0));

  // for now fixed distortion values TODO should be set dynamically by OVR
  set_distortion(1.0, 0.22, 0.24, 0.0);

}

////////////////////////////////////////////////////////////////////////////////

OculusWindow::~OculusWindow() {

}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::create_shader() {
  fullscreen_shader_.create_from_sources(R"(
      #version 420
      #extension GL_NV_bindless_texture : require

      layout(location=0) in vec3 in_position;

      out vec2 tex_coord;

      void main() {
          tex_coord = in_position.xy*0.5 + 0.5;
          gl_Position = vec4(in_position, 1.0);
      }
    )", R"(
      #version 420
      #extension GL_NV_bindless_texture : require
      #extension GL_NV_gpu_shader5      : enable

      in vec2 tex_coord;

      uniform uvec2 sampler;

      // oculus parameters
      uniform vec2 lens_center;
      uniform vec2 scale;
      uniform vec4 hmd_warp_param;

      layout (location = 0) out vec3 out_color;

      sampler2D get_tex(uvec2 handle) {
          return sampler2D(uint64_t(handle.x) + uint64_t(handle.y) * 4294967295);
      }

      vec2 hmd_warp(vec2 in_texcoord) {
          vec2 theta = (in_texcoord - lens_center) * 2.0; // Scales to [-1, 1]
          float rSq = theta.x * theta.x + theta.y * theta.y;
          vec2 rvector = theta * (hmd_warp_param.x+hmd_warp_param.y*rSq
                                                  +hmd_warp_param.z*rSq*rSq
                                                  +hmd_warp_param.w*rSq*rSq*rSq);
          return lens_center + scale * rvector;
      }

      vec3 get_color() {

          vec2 tc = hmd_warp(tex_coord);

          if (tc.x < 0.0 || tc.y < 0.0 || tc.x > 1.0 || tc.y > 1.0 )
              return vec3(0);

          return vec3(texture2D( get_tex(sampler), tc).rgb);
      }

      void main() {
          out_color = get_color();
      }
    )");
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::set_distortion(math::vec4 const& distortion) {
  distortion_ = distortion;
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::set_distortion(float distortion0, float distortion1, float distortion2, float distortion3) {
  distortion_[0] = distortion0;
  distortion_[1] = distortion1;
  distortion_[2] = distortion2;
  distortion_[3] = distortion3;
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::display(std::shared_ptr<Texture2D> const& left_texture,
                           std::shared_ptr<Texture2D> const& right_texture) {

  display(left_texture, config.get_left_resolution(), config.get_left_position(), true);
  display(right_texture, config.get_right_resolution(), config.get_right_position(), false);
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::display(std::shared_ptr<Texture2D> const& texture,
                     math::vec2ui const& size,
                     math::vec2ui const& position,
                     bool left) {


  fullscreen_shader_.use(*get_context());
  fullscreen_shader_.set_uniform(*get_context(), texture, "sampler");

  if (left) fullscreen_shader_.set_uniform(*get_context(), math::vec2(0.6f, 0.5f), "lens_center");
  else      fullscreen_shader_.set_uniform(*get_context(), math::vec2(0.4f, 0.5f), "lens_center");

  fullscreen_shader_.set_uniform(*get_context(), math::vec2(0.4f, 0.4f), "scale");
  fullscreen_shader_.set_uniform(*get_context(), distortion_, "hmd_warp_param");

  get_context()->render_context->set_viewport(scm::gl::viewport(position, size));
  get_context()->render_context->set_depth_stencil_state(depth_stencil_state_);

  fullscreen_quad_->draw(get_context()->render_context);

  get_context()->render_context->reset_state_objects();
  fullscreen_shader_.unuse(*get_context());
}

}
