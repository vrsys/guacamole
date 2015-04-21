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

#include <gua/config.hpp>
#include <gua/node/TriMeshNode.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/TriMeshRessource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

#include <scm/core/math/math.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TriMeshRenderer::TriMeshRenderer()
{
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
  ResourceFactory factory;
  std::string v_shader = factory.read_shader_file("resources/shaders/tri_mesh_shader.vert");
  std::string f_shader = factory.read_shader_file("resources/shaders/tri_mesh_shader.frag");
#else
  std::string v_shader = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
  std::string f_shader = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
#endif

  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
  program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRenderer::create_state_objects(RenderContext const& ctx)
{
  rs_cull_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
  rs_cull_none_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc, bool rendering_shadows)
{
  auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::TriMeshNode))));
  if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

    std::sort(sorted_objects->second.begin(), sorted_objects->second.end(),
              [](node::Node* a, node::Node* b) {
                return reinterpret_cast<node::TriMeshNode*>(a)->get_material()->get_shader()
                     < reinterpret_cast<node::TriMeshNode*>(b)->get_material()->get_shader();
              });

    RenderContext const& ctx(pipe.get_context());

    std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.get_scene_camera().uuid) + " / TrimeshPass";
    std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.get_scene_camera().uuid) + " / TrimeshPass";

    pipe.begin_gpu_query(ctx, gpu_query_name);
    pipe.begin_cpu_query(cpu_query_name);

    bool write_depth = true;
    pipe.get_current_target().bind(ctx, write_depth);
    pipe.get_current_target().set_viewport(ctx);

    int view_id(pipe.get_scene_camera().config.get_view_id());

    MaterialShader*                current_material(nullptr);
    std::shared_ptr<ShaderProgram> current_shader;
    auto current_rasterizer_state = rs_cull_back_;
    ctx.render_context->apply();

    // loop through all objects, sorted by material ----------------------------
    for (auto const& object : sorted_objects->second) {

      auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));
      if (rendering_shadows && tri_mesh_node->get_shadow_mode() == ShadowMode::OFF) {
        continue;
      }

      if (current_material != tri_mesh_node->get_material()->get_shader()) {
        current_material = tri_mesh_node->get_material()->get_shader();
        if (current_material) {

          auto shader_iterator = programs_.find(current_material);
          if (shader_iterator != programs_.end())
          {
            current_shader = shader_iterator->second;
          }
          else {
            auto smap = global_substitution_map_;
            for (const auto& i: current_material->generate_substitution_map())
              smap[i.first] = i.second;

            current_shader = std::make_shared<ShaderProgram>();
            current_shader->set_shaders(program_stages_, std::list<std::string>(), false, smap);
            programs_[current_material] = current_shader;
          }
        }
        else {
          Logger::LOG_WARNING << "TriMeshPass::process(): Cannot find material: "
                              << tri_mesh_node->get_material()->get_shader_name() << std::endl;
        }
        if (current_shader) {
          current_shader->use(ctx);
          current_shader->set_uniform(ctx, math::vec2i(pipe.get_current_target().get_width(),
                                                       pipe.get_current_target().get_height()),
                                      "gua_resolution"); //TODO: pass gua_resolution. Probably should be somehow else implemented
          current_shader->set_uniform(ctx, 1.0f / pipe.get_current_target().get_width(),  "gua_texel_width");
          current_shader->set_uniform(ctx, 1.0f / pipe.get_current_target().get_height(), "gua_texel_height");
          // hack
          current_shader->set_uniform(ctx, pipe.get_current_target().get_depth_buffer()->get_handle(ctx),
                                      "gua_gbuffer_depth");
        }
      }

      if (current_shader && tri_mesh_node->get_geometry())
      {
        auto model_view_mat = pipe.get_scene().frustum.get_view() * tri_mesh_node->get_cached_world_transform();
        UniformValue normal_mat (math::mat4f(scm::math::transpose(scm::math::inverse(tri_mesh_node->get_cached_world_transform()))));

        int rendering_mode = rendering_shadows ? (tri_mesh_node->get_shadow_mode() == ShadowMode::HIGH_QUALITY ? 2 : 1) : 0;

        current_shader->apply_uniform(ctx, "gua_model_matrix", math::mat4f(tri_mesh_node->get_cached_world_transform()));
        current_shader->apply_uniform(ctx, "gua_model_view_matrix", math::mat4f(model_view_mat));
        current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);
        current_shader->apply_uniform(ctx, "gua_rendering_mode", rendering_mode);

        // lowfi shadows dont need material input
        if (rendering_mode != 1) {
          tri_mesh_node->get_material()->apply_uniforms(ctx, current_shader.get(), view_id);
        }

        current_rasterizer_state = tri_mesh_node->get_material()->get_show_back_faces() ? rs_cull_none_ : rs_cull_back_;

        if (ctx.render_context->current_rasterizer_state() != current_rasterizer_state) {
          ctx.render_context->set_rasterizer_state(current_rasterizer_state);
          ctx.render_context->apply_state_objects();
        }

        ctx.render_context->apply_program();

        tri_mesh_node->get_geometry()->draw(ctx);
      }
    }

    pipe.get_current_target().unbind(ctx);

    pipe.end_gpu_query(ctx, gpu_query_name);
    pipe.end_cpu_query(cpu_query_name);

    ctx.render_context->reset_state_objects();
  }

}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
