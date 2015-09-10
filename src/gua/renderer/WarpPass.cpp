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
  , debug_sample_ray_(false)
  , debug_interpolation_borders_(false)
  , debug_rubber_bands_(false)
  , max_layers_(50)
  , pixel_size_(0.0f)
  , rubber_band_threshold_(0.01f)
  , gbuffer_warp_mode_(GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION)
  , abuffer_warp_mode_(ABUFFER_RAYCASTING)
  , hole_filling_mode_(HOLE_FILLING_INPAINT)
  , interpolation_mode_(INTERPOLATION_MODE_ADAPTIVE)
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "WarpPass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = false;
  rendermode_ = RenderMode::Custom;
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

WarpPassDescription& WarpPassDescription::debug_sample_ray(bool val) {
  debug_sample_ray_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::debug_sample_ray() const {
  return debug_sample_ray_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::debug_interpolation_borders(bool val) {
  debug_interpolation_borders_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::debug_interpolation_borders() const {
  return debug_interpolation_borders_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::debug_rubber_bands(bool val) {
  debug_rubber_bands_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool WarpPassDescription::debug_rubber_bands() const {
  return debug_rubber_bands_;
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

WarpPassDescription& WarpPassDescription::rubber_band_threshold(float val) {
  rubber_band_threshold_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float WarpPassDescription::rubber_band_threshold() const {
  return rubber_band_threshold_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::gbuffer_warp_mode(GBufferWarpMode gbuffer_warp_mode) {
  gbuffer_warp_mode_ = gbuffer_warp_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::GBufferWarpMode WarpPassDescription::gbuffer_warp_mode() const {
  return gbuffer_warp_mode_;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription& WarpPassDescription::abuffer_warp_mode(ABufferWarpMode abuffer_warp_mode) {
  abuffer_warp_mode_ = abuffer_warp_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::ABufferWarpMode WarpPassDescription::abuffer_warp_mode() const {
  return abuffer_warp_mode_;
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

WarpPassDescription& WarpPassDescription::interpolation_mode(InterpolationMode interpolation_mode) {
  interpolation_mode_ = interpolation_mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::InterpolationMode WarpPassDescription::interpolation_mode() const {
  return interpolation_mode_;
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
  substitution_map["debug_interpolation_borders"] = debug_interpolation_borders_ ? "1" : "0";
  substitution_map["debug_rubber_bands"] = debug_rubber_bands_ ? "1" : "0";
  substitution_map["debug_sample_ray"] = debug_sample_ray_ ? "1" : "0";
  substitution_map["pixel_size"] = gua::string_utils::to_string(pixel_size_);
  substitution_map["rubber_band_threshold"] = gua::string_utils::to_string(rubber_band_threshold_);
  substitution_map["gbuffer_warp_mode"] = std::to_string(gbuffer_warp_mode_);
  substitution_map["abuffer_warp_mode"] = std::to_string(abuffer_warp_mode_);
  substitution_map["hole_filling_mode"] = std::to_string(hole_filling_mode_);
  substitution_map["interpolation_mode"] = std::to_string(interpolation_mode_);
  substitution_map["warping_max_layers"] = std::to_string(max_layers_);
  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer = std::make_shared<WarpRenderer>();
  renderer->set_global_substitution_map(substitution_map);

  pass.process_ = [renderer](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {
    renderer->render(pipe, desc);
  };

  return pass;
}

}
