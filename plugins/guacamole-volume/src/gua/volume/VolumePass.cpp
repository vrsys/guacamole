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
#include <gua/volume/VolumePass.hpp>

#include <gua/volume/VolumeNode.hpp>
#include <gua/volume/VolumeRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>

namespace gua {

VolumePassDescription::VolumePassDescription()
  : PipelinePassDescription() {

  vertex_shader_ = "shaders/textured_screen_space_quad.vert";
  fragment_shader_ = "shaders/textured_screen_space_quad.frag";

  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = true;
  doClear_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass VolumePassDescription::make_pass(RenderContext const& ctx,
    SubstitutionMap& substitution_map) {

  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer(std::make_shared<VolumeRenderer>());

  pass.process_ = [renderer](
      PipelinePass & pass, PipelinePassDescription const&, Pipeline& pipe) {

      renderer->render(pipe);
  };

  return pass;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* VolumePassDescription::make_copy() const {
  return new VolumePassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

}
