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
#include <gua/renderer/TriMeshPass.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/TriMeshRenderer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/node/TriMeshNode.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TriMeshPassDescription::TriMeshPassDescription()
  : PipelinePassDescription() {
  vertex_shader_ = ""; // "shaders/tri_mesh_shader.vert";
  fragment_shader_ = ""; // "shaders/tri_mesh_shader.frag";
  name_ = "TriMeshPass";

  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = false;
  enable_for_shadows_ = true;
  rendermode_ = RenderMode::Custom;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(true, true));
}


////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<PipelinePassDescription> TriMeshPassDescription::make_copy() const {
  return std::make_shared<TriMeshPassDescription>(*this);
}


////////////////////////////////////////////////////////////////////////////////

PipelinePass TriMeshPassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer = std::make_shared<TriMeshRenderer>();
  renderer->set_global_substitution_map(substitution_map);
  renderer->create_state_objects(ctx);

  pass.process_ = [renderer](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe, bool rendering_shadows) {

    pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state_);
    renderer->render(pipe, desc);
  };

  return pass;
}

}
