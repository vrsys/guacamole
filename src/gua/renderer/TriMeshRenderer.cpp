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
#include <gua/renderer/TriMeshRenderer.hpp>

#include <gua/node/TriMeshNode.hpp>

#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  TriMeshRenderer::TriMeshRenderer()
    : program_description_(),
      programs_()
  {
    program_description_[scm::gl::shader_stage::STAGE_VERTEX_SHADER] = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
    program_description_[scm::gl::shader_stage::STAGE_FRAGMENT_SHADER] = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
  }


  ////////////////////////////////////////////////////////////////////////////////

  TriMeshRenderer::~TriMeshRenderer()
  {
    for (auto program : programs_) {
      delete program.second;
    }
    programs_.clear();
  }


  ////////////////////////////////////////////////////////////////////////////////

  void TriMeshRenderer::render(Pipeline& pipe)
  {
    auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::TriMeshNode))));

    if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

      std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b){
        return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
      });

      RenderContext const& ctx(pipe.get_context());

      bool writes_only_color_buffer = false;
      pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);
      pipe.get_gbuffer().set_viewport(ctx);

      int view_id(pipe.get_camera().config.get_view_id());

      MaterialShader* current_material(nullptr);
      ShaderProgram*  current_shader(nullptr);

      ctx.render_context->apply();

      // loop through all objects, sorted by material ----------------------------
      for (auto const& object : sorted_objects->second) {

        auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));

        if (current_material != tri_mesh_node->get_material()->get_shader()) {          
          current_material = tri_mesh_node->get_material()->get_shader();
          if (current_material) {

            auto shader_iterator = programs_.find(current_material);
            if (shader_iterator != programs_.end())
            {
              current_shader = shader_iterator->second;
            }
            else {
              auto shader = current_material->get_shader(program_description_);
              programs_[current_material] = shader;
              current_shader = shader;
            }           
          }
          else {
            Logger::LOG_WARNING << "TriMeshPass::process(): Cannot find material: " << tri_mesh_node->get_material()->get_shader_name() << std::endl;
          }
          if (current_shader) {
            current_shader->use(ctx);
          }
        }

        if (current_shader && tri_mesh_node->get_geometry()) {
          UniformValue model_mat(tri_mesh_node->get_cached_world_transform());
          UniformValue normal_mat(scm::math::transpose(scm::math::inverse(tri_mesh_node->get_cached_world_transform())));

          current_shader->apply_uniform(ctx, "gua_model_matrix", model_mat);
          current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

          tri_mesh_node->get_material()->apply_uniforms(ctx, current_shader, view_id);

          ctx.render_context->apply_program();

          tri_mesh_node->get_geometry()->draw(ctx);
        }
      }

      pipe.get_gbuffer().unbind(ctx);
    }
  }
} // namespace gua
