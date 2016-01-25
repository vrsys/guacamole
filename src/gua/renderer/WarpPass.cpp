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
  , depth_test_(true)
  , adaptive_entry_level_(true)
  , debug_cell_colors_(false)
  , debug_sample_count_(false)
  , debug_bounding_volumes_(false)
  , hole_filling_color_(0,0,0)
  , max_raysteps_(50)
  , pixel_size_(0.2f)
  , hole_filling_mode_(HOLE_FILLING_BLUR)
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "WarpPass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::get_warp_state(std::function<WarpPassDescription::WarpState()> const& f) {
  get_warp_state_ = f;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::function<WarpPassDescription::WarpState()> const& WarpPassDescription::get_warp_state() const {
  return get_warp_state_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::max_raysteps(int val) {
  max_raysteps_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

int WarpPassDescription::max_raysteps() const {
  return max_raysteps_;
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

WarpPassDescription& WarpPassDescription::adaptive_entry_level(bool val) {
  adaptive_entry_level_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::adaptive_entry_level() const {
  return adaptive_entry_level_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::debug_cell_colors(bool val) {
  debug_cell_colors_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::debug_cell_colors() const {
  return debug_cell_colors_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::debug_sample_count(bool val) {
  debug_sample_count_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::debug_sample_count() const {
  return debug_sample_count_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::debug_bounding_volumes(bool val) {
  debug_bounding_volumes_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::debug_bounding_volumes() const {
  return debug_bounding_volumes_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::pixel_size(float val) {
  pixel_size_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float WarpPassDescription::pixel_size() const {
  return pixel_size_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::hole_filling_mode(HoleFillingMode hole_filling_mode) {
  hole_filling_mode_ = hole_filling_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::HoleFillingMode WarpPassDescription::hole_filling_mode() const {
  return hole_filling_mode_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::hole_filling_color(math::vec3f const& hole_filling_color) {
  hole_filling_color_ = hole_filling_color;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

math::vec3f const& WarpPassDescription::hole_filling_color() const {
  return hole_filling_color_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> WarpPassDescription::make_copy() const {
  return std::make_shared<WarpPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass WarpPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {
  substitution_map["gua_debug_tiles"] = "0";
  substitution_map["adaptive_entry_level"] = adaptive_entry_level_ ? "1" : "0";
  substitution_map["debug_cell_colors"] = debug_cell_colors_ ? "1" : "0";
  substitution_map["debug_sample_count"] = debug_sample_count_ ? "1" : "0";
  substitution_map["debug_bounding_volumes"] = debug_bounding_volumes_ ? "1" : "0";
  substitution_map["pixel_size"] = gua::string_utils::to_string(pixel_size_);
  substitution_map["hole_filling_mode"] = std::to_string(hole_filling_mode_);
  substitution_map["hole_filling_color"] = "vec3(" + gua::string_utils::to_string(hole_filling_color_.x) + ", " + gua::string_utils::to_string(hole_filling_color_.y) + ", " + gua::string_utils::to_string(hole_filling_color_.z) + ")";
  substitution_map["max_raysteps"] = std::to_string(max_raysteps_);
  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer = std::make_shared<WarpRenderer>();
  renderer->set_global_substitution_map(substitution_map);

  pass.process_ = [renderer](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {

    if (pipe.current_viewstate().camera.config.get_stereo_type() == StereoType::SPATIAL_WARP ||
        pipe.current_viewstate().camera.config.get_stereo_type() == StereoType::TEMPORAL_WARP) {
      renderer->render(pipe, desc);
    }
  };

  return pass;
}

}
