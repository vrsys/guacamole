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
#include <gua/renderer/NURBSPass.hpp>

// guacamole headers
#include <gua/renderer/NURBSResource.hpp>
#include <gua/renderer/NURBSRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>
#include <gpucast/core/config.hpp>

#include <scm/gl_core/shader_objects.h>

// external headers
#include <sstream>
#include <fstream>
#include <regex>
#include <list>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

NURBSPassDescription::NURBSPassDescription()
  : PipelinePassDescription()
{
  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = false;
  doClear_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* NURBSPassDescription::make_copy() const {
  return new NURBSPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass NURBSPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{ *this, ctx, substitution_map };

  auto renderer = std::make_shared<NURBSRenderer>();
  renderer->set_global_substitution_map(substitution_map);

  pass.process_ = [renderer](
    PipelinePass&, PipelinePassDescription const& desc, Pipeline & pipe) {
    renderer->render(pipe, desc);
  };

  return pass;
}

}
