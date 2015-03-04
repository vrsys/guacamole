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
#include <gua/renderer/SkeletalAnimationRenderer.hpp>

#include <gua/node/SkeletalAnimationNode.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  SkeletalAnimationRenderer::SkeletalAnimationRenderer(RenderContext const& ctx)
    : bones_block_(ctx.render_device)
  {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    ResourceFactory factory;
    std::string v_shader = factory.read_shader_file("resources/shaders/skinned_mesh_shader.vert");
    std::string f_shader = factory.read_shader_file("resources/shaders/skinned_mesh_shader.frag");
#else
    std::string v_shader = Resources::lookup_shader("shaders/skinned_mesh_shader.vert");
    std::string f_shader = Resources::lookup_shader("shaders/skinned_mesh_shader.frag");
#endif

    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_shader));
    program_stages_.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_shader));
  }

  ////////////////////////////////////////////////////////////////////////////////

  void SkeletalAnimationRenderer::create_state_objects(RenderContext const& ctx)
  {
    rs_cull_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
    rs_cull_none_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
  }

  ////////////////////////////////////////////////////////////////////////////////

  void SkeletalAnimationRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
  {
    auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::SkeletalAnimationNode))));

    if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

      // TODO
      /*std::sort(sorted_objects->second.begin(), sorted_objects->second.end(),
                [](node::Node* a, node::Node* b) {
                  return reinterpret_cast<node::SkeletalAnimationNode*>(a)->get_material()->get_shader()
                       < reinterpret_cast<node::SkeletalAnimationNode*>(b)->get_material()->get_shader();
                });*/

      RenderContext const& ctx(pipe.get_context());

      bool writes_only_color_buffer = false;
      pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);
      pipe.get_gbuffer().set_viewport(ctx);
      pipe.get_abuffer().bind(ctx);

      int view_id(pipe.get_camera().config.get_view_id());

      MaterialShader*                current_material(nullptr);
      std::shared_ptr<ShaderProgram> current_shader;
      auto current_rasterizer_state = rs_cull_back_;
      ctx.render_context->apply();

      // loop through all objects, sorted by material ----------------------------
      for (auto const& object : sorted_objects->second) {

        auto skel_anim_node(reinterpret_cast<node::SkeletalAnimationNode*>(object));


        auto materials = skel_anim_node->get_materials();
        auto geometries = skel_anim_node->get_geometries();

        // loop through all resources in animation node
        for(uint i(0);i<materials.size();++i){

          if (current_material != materials[i]->get_shader()) {
            current_material = materials[i]->get_shader();
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
              Logger::LOG_WARNING << "SkeletalAnimationPass::process(): Cannot find material: "
                                  << materials[i]->get_shader_name() << std::endl;
            }
            if (current_shader) {
              current_shader->use(ctx);
              current_shader->set_uniform(ctx, math::vec2i(pipe.get_gbuffer().get_width(),
                                                           pipe.get_gbuffer().get_height()),
                                          "gua_resolution"); //TODO: pass gua_resolution. Probably should be somehow else implemented
              current_shader->set_uniform(ctx, 1.0f / pipe.get_gbuffer().get_width(),  "gua_texel_width");
              current_shader->set_uniform(ctx, 1.0f / pipe.get_gbuffer().get_height(), "gua_texel_height");
            }
          }

          if (current_shader && geometries[i]) {
            UniformValue model_mat(::scm::math::mat4f(skel_anim_node->get_cached_world_transform()));
            UniformValue normal_mat(::scm::math::mat4f(scm::math::transpose(scm::math::inverse(skel_anim_node->get_cached_world_transform()))));

            current_shader->apply_uniform(ctx, "gua_model_matrix", model_mat);
            current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

            materials[i]->apply_uniforms(ctx, current_shader.get(), view_id);

            current_rasterizer_state = materials[i]->get_show_back_faces() ? rs_cull_none_ : rs_cull_back_;

            if (ctx.render_context->current_rasterizer_state() != current_rasterizer_state) {
              ctx.render_context->set_rasterizer_state(current_rasterizer_state);
              ctx.render_context->apply_state_objects();
            }

            ctx.render_context->apply_program();

            bones_block_.update(ctx.render_context, skel_anim_node->get_director()->get_bone_transforms());

            ctx.render_context->bind_uniform_buffer( bones_block_.block().block_buffer(), 2);

            geometries[i]->draw(ctx);
          }
        }
      }

      pipe.get_gbuffer().unbind(ctx);
      pipe.get_abuffer().unbind(ctx);
      ctx.render_context->reset_state_objects();
    }

  }
} // namespace gua
