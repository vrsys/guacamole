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
#include <gua/renderer/GenerateWarpGridPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/WarpGridGenerator.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
GenerateWarpGridPassDescription::GenerateWarpGridPassDescription()
  : PipelinePassDescription()
  , cell_size_(32)
  , mode_(WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION)
  , split_threshold_(0.0005f)
  , max_split_depth_(1.f)
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "GenerateWarpGridPass";
  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

GenerateWarpGridPassDescription& GenerateWarpGridPassDescription::cell_size(int val) {
  cell_size_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

int GenerateWarpGridPassDescription::cell_size() const {
  return cell_size_;
}

////////////////////////////////////////////////////////////////////////////////

GenerateWarpGridPassDescription& GenerateWarpGridPassDescription::split_threshold(float val) {
  split_threshold_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float GenerateWarpGridPassDescription::split_threshold() const {
  return split_threshold_;
}

////////////////////////////////////////////////////////////////////////////////

GenerateWarpGridPassDescription& GenerateWarpGridPassDescription::max_split_depth(float val) {
  max_split_depth_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

float GenerateWarpGridPassDescription::max_split_depth() const {
  return max_split_depth_;
}

////////////////////////////////////////////////////////////////////////////////

GenerateWarpGridPassDescription& GenerateWarpGridPassDescription::mode(WarpPassDescription::GBufferWarpMode mode) {
  mode_ = mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::GBufferWarpMode GenerateWarpGridPassDescription::mode() const {
  return mode_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> GenerateWarpGridPassDescription::make_copy() const {
  return std::make_shared<GenerateWarpGridPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass GenerateWarpGridPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {

  PipelinePass pass{*this, ctx, substitution_map};

  substitution_map["gbuffer_warp_mode"] = std::to_string(mode_);
  substitution_map["split_threshold"] = gua::string_utils::to_string(split_threshold_);
  substitution_map["max_split_depth"] = gua::string_utils::to_string(max_split_depth_);
  substitution_map["use_max_split_depth"] = (max_split_depth_ == 1.0) ? "0" : "1";

  auto renderer = std::make_shared<WarpGridGenerator>();
  renderer->set_global_substitution_map(substitution_map);

  WarpPassDescription::GBufferWarpMode mode(mode_);

  pass.process_ = [renderer, mode](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {
    if (mode == WarpPassDescription::GBUFFER_GRID_DEPTH_THRESHOLD ||
        mode == WarpPassDescription::GBUFFER_GRID_SURFACE_ESTIMATION ||
        mode == WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION ||
        mode == WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION) {
      renderer->render(pipe, desc);
    }
  };

  return pass;
}

}
