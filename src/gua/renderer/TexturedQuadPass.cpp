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
#include <gua/renderer/TexturedQuadPass.hpp>

#include <gua/node/TexturedQuadNode.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>

#define USE_UBO 0 // also set in MaterialShader.cpp

namespace gua {

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* TexturedQuadPassDescription::make_copy() const {
  return new TexturedQuadPassDescription(*this);
}


////////////////////////////////////////////////////////////////////////////////

PipelinePass* TexturedQuadPassDescription::make_pass() const {
  return new TexturedQuadPass();
}

////////////////////////////////////////////////////////////////////////////////

TexturedQuadPass::TexturedQuadPass() {
  // shader_.create_from_sources(
  //   Resources::lookup_shader(Resources::shaders_textured_quad_vert), 
  //   Resources::lookup_shader(Resources::shaders_textured_quad_frag)
  // );
}

////////////////////////////////////////////////////////////////////////////////

void TexturedQuadPass::process(PipelinePassDescription* desc, Pipeline* pipe) {
  for (auto const& node : pipe->get_scene().nodes[std::type_index(typeid(node::TexturedQuadNode))]) {
    auto quad_node(reinterpret_cast<node::TexturedQuadNode*>(node));

    // shader_.set_uniform(ctx, quad_node->get_scaled_world_transform(), "gua_model_matrix");
    // shader_.set_uniform(ctx, quad_node->data.get_falloff(),           "gua_texture");
    
    // pipe->draw_quad();
  }
}

////////////////////////////////////////////////////////////////////////////////

}
