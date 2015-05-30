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
#include <gua/renderer/WarpPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/WarpRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
WarpPassDescription::WarpPassDescription()
  : PipelinePassDescription()
  , max_layers_(10)
  , depth_test_(true)
  , mode_(POINTS)
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "WarpPass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Custom;

  uniforms["original_inverse_projection_view_matrix"] = scm::math::make_translation(0.f, 0.f, 0.f);
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::use_abuffer_from_window(std::string const& name) {
  shared_window_name_ = name;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& WarpPassDescription::use_abuffer_from_window() const {
  return shared_window_name_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::original_inverse_projection_view_matrix(math::mat4f const& mat) {
  uniforms["original_inverse_projection_view_matrix"] = mat;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

math::mat4f const& WarpPassDescription::original_inverse_projection_view_matrix() const {
  auto uniform(uniforms.find("original_inverse_projection_view_matrix"));
  return boost::get<math::mat4f>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::max_layers(int val) {
  max_layers_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

int WarpPassDescription::max_layers() const {
  return max_layers_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::depth_test(bool val) {
  depth_test_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::depth_test() const {
  return depth_test_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::display_mode(DisplayMode mode) {
  mode_ = mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::DisplayMode WarpPassDescription::display_mode() const {
  return mode_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> WarpPassDescription::make_copy() const {
  return std::make_shared<WarpPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass WarpPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {
  substitution_map["gua_debug_tiles"] = "0";
  substitution_map["display_mode"] = std::to_string(mode_);
  substitution_map["warping_max_layers"] = std::to_string(max_layers_);
  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer = std::make_shared<WarpRenderer>();
  renderer->set_global_substitution_map(substitution_map);

  pass.process_ = [renderer](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {

    pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state_, 1);
    renderer->render(pipe, desc);
  };

  return pass;
}

}
