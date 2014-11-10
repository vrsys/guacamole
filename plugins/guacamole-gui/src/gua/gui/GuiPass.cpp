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
#include <gua/gui/GuiPass.hpp>

#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/gui/GuiResource.hpp>
#include <gua/gui/GuiNode.hpp>

namespace gua {
namespace gui {

////////////////////////////////////////////////////////////////////////////////


PipelinePassDescription* GuiPassDescription::make_copy() const {
  return new GuiPassDescription(*this);
}


////////////////////////////////////////////////////////////////////////////////

PipelinePass* GuiPassDescription::make_pass() const {
  return new GuiPass();
}

////////////////////////////////////////////////////////////////////////////////

GuiPass::GuiPass() :
  shader_(new ShaderProgram()),
  fullscreen_quad_(nullptr) {

  std::string const v_shader(
    R"(
      // vertex shader ---------------------------------------------------------
      @include "shaders/common/header.glsl"
      @include "shaders/common/gua_camera_uniforms.glsl"

      // input
      layout(location=0) in vec3 position;

      // uniforms
      // uniform vec2 size;
      // uniform vec2 offset;

      // varyings
      out vec2 tex_coords;

      void main(void) {
        tex_coords  = vec2(position.x + 1.0, 1.0 - position.y) * 0.5;
        // vec2 pos    = position*size + offset;
        gl_Position = vec4(position, 1.0);
      }
    )"
  );

  std::string const f_shader(
    R"(
      // fragment shader -------------------------------------------------------
      @include "shaders/common/header.glsl"

      // input
      in vec2 tex_coords;

      // uniforms
      uniform sampler2D gua_gui_diffuse_tex;

      // output
      @include "shaders/common/gua_fragment_shader_output.glsl"


      void main(void){
        vec4 color = texture(diffuse, tex_coords);
        color.rgb /= color.a;

        gua_out_color = color;
      }
    )"
  );

  Resources::resolve_includes(v_shader);
  Resources::resolve_includes(f_shader);

  shader_->create_from_sources(v_shader, f_shader);
}

////////////////////////////////////////////////////////////////////////////////

void GuiPass::process(PipelinePassDescription* desc, Pipeline* pipe) {

  auto gui_nodes(pipe->get_scene().nodes.find(std::type_index(typeid(gui::GuiNode))));

  if (gui_nodes != pipe->get_scene().nodes.end() && gui_nodes.size() > 0) {

    auto const& ctx(pipe->get_context());

    if (!quad_) {
      quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
              get_context().render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
    }

    pipe->get_gbuffer().bind(ctx, this);
    pipe->get_gbuffer().set_viewport(ctx);

    for (auto const& node : gui_nodes) {

      shader_->use(ctx);
      ctx.render_context->apply();

      auto const& gui_node(reinterpret_cast<gui::GuiNode*>(node));

      UniformValue model_mat(gui_node->get_cached_world_transform());
      // UniformValue normal_mat(scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())));


      shader->apply_uniform(ctx, "gua_model_matrix", model_mat);
      shader->apply_uniform(ctx, "gua_gui_diffuse_tex", 0);

      ctx.render_context->apply_program();

      gui_node->get_resource()->bind(ctx, shader);
      quad_->draw(get_context().render_context);
    }

    pipe->get_gbuffer().unbind(ctx);
  }

}

////////////////////////////////////////////////////////////////////////////////

}
}

