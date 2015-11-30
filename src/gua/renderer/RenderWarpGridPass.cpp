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
#include <gua/renderer/RenderWarpGridPass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/WarpGridRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
RenderWarpGridPassDescription::RenderWarpGridPassDescription()
  : PipelinePassDescription()
  , show_warp_grid_(false)
  , mode_(WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION)
{
  vertex_shader_ = "";
  fragment_shader_ = "";
  name_ = "RenderWarpGridPass";
  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

RenderWarpGridPassDescription& RenderWarpGridPassDescription::show_warp_grid(bool val) {
  show_warp_grid_ = val;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

bool RenderWarpGridPassDescription::show_warp_grid() const {
  return show_warp_grid_;
}

////////////////////////////////////////////////////////////////////////////////

RenderWarpGridPassDescription& RenderWarpGridPassDescription::mode(WarpPassDescription::GBufferWarpMode mode) {
  mode_ = mode;
  touch();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

WarpPassDescription::GBufferWarpMode RenderWarpGridPassDescription::mode() const {
  return mode_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> RenderWarpGridPassDescription::make_copy() const {
  return std::make_shared<RenderWarpGridPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass RenderWarpGridPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {

  substitution_map["gbuffer_warp_mode"] = std::to_string(mode_);

  auto grid_renderer = std::make_shared<WarpGridRenderer>();
  PipelinePass pass{*this, ctx, substitution_map};

  WarpPassDescription::GBufferWarpMode mode(mode_);

  pass.process_ = [grid_renderer, mode](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {

    auto description(dynamic_cast<RenderWarpGridPassDescription const*>(&desc));

    if (pipe.current_viewstate().camera.config.get_stereo_type() == StereoType::SPATIAL_WARP ||
        pipe.current_viewstate().camera.config.get_stereo_type() == StereoType::TEMPORAL_WARP ||
        pipe.current_viewstate().camera.config.get_stereo_type() == StereoType::SINGLE_TEMPORAL_WARP) {
      if (description->show_warp_grid() && (
          mode == WarpPassDescription::GBUFFER_GRID_DEPTH_THRESHOLD ||
          mode == WarpPassDescription::GBUFFER_GRID_SURFACE_ESTIMATION ||
          mode == WarpPassDescription::GBUFFER_GRID_NON_UNIFORM_SURFACE_ESTIMATION ||
          mode == WarpPassDescription::GBUFFER_GRID_ADVANCED_SURFACE_ESTIMATION)) {
        grid_renderer->render(pipe, desc);
      }
    }
  };

  return pass;
}

}
