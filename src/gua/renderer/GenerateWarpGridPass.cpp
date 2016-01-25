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
  , split_threshold_(0.0001f)
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

std::shared_ptr<PipelinePassDescription> GenerateWarpGridPassDescription::make_copy() const {
  return std::make_shared<GenerateWarpGridPassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass GenerateWarpGridPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {

  PipelinePass pass{*this, ctx, substitution_map};

  substitution_map["split_threshold"] = gua::string_utils::to_string(split_threshold_);

  auto renderer = std::make_shared<WarpGridGenerator>();
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
