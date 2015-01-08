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

#include <gua/renderer/SkeletalAnimationRessource.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>

#include <gua/databases/Resources.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

<<<<<<< HEAD
  SkeletalAnimationRenderer::SkeletalAnimationRenderer()
    : program_description_(),
      programs_()
  {
    program_description_[scm::gl::shader_stage::STAGE_VERTEX_SHADER] = Resources::lookup_shader("shaders/skeletal_animation_shader.vert");
    program_description_[scm::gl::shader_stage::STAGE_FRAGMENT_SHADER] = Resources::lookup_shader("shaders/skeletal_animation_shader.frag");
=======
  SkeletalAnimationRenderer::SkeletalAnimationRenderer(RenderContext const& ctx)
    : program_factory_(),
      program_description_(),
      programs_(),
      bones_block_(ctx.render_device)
  {
#ifdef GUACAMOLE_RUNTIME_PROGRAM_COMPILATION
    program_description_[scm::gl::shader_stage::STAGE_VERTEX_SHADER]   = program_factory_.read_from_file("resources/shaders/skeletal_animation_shader.vert");
    program_description_[scm::gl::shader_stage::STAGE_FRAGMENT_SHADER] = program_factory_.read_from_file("resources/shaders/skeletal_animation_shader.frag");
#else
    program_description_[scm::gl::shader_stage::STAGE_VERTEX_SHADER] = Resources::lookup_shader("shaders/skeletal_animation_shader.vert");
    program_description_[scm::gl::shader_stage::STAGE_FRAGMENT_SHADER] = Resources::lookup_shader("shaders/skeletal_animation_shader.frag");
#endif
>>>>>>> Create one UBO.
  }


  ////////////////////////////////////////////////////////////////////////////////

  SkeletalAnimationRenderer::~SkeletalAnimationRenderer()
  {
    for (auto program : programs_) {
      delete program.second;
    }
    programs_.clear();
  }

  ////////////////////////////////////////////////////////////////////////////////

  void SkeletalAnimationRenderer::render(Pipeline& pipe)
  {
    auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::SkeletalAnimationNode))));

    if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

      //TODO!!!!!
      /*std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b){
        return reinterpret_cast<node::SkeletalAnimationNode*>(a)->get_material().get_shader() < reinterpret_cast<node::SkeletalAnimationNode*>(b)->get_material().get_shader();
      });*/


      //RenderContext ctx(pipe.get_context());
      RenderContext & ctx(pipe.get_context());

      bool writes_only_color_buffer = false;
      pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);
      pipe.get_gbuffer().set_viewport(ctx);

      int view_id(pipe.get_camera().config.get_view_id());

      MaterialShader* current_material(nullptr);
      ShaderProgram*  current_shader(nullptr);

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
                auto shader = current_material->get_shader(program_description_);
                programs_[current_material] = shader;
                current_shader = shader;
              }           
            }
            else {
              Logger::LOG_WARNING << "SkeletalAnimationPass::process(): Cannot find material: " << materials[i]->get_shader_name() << std::endl;
            }

            if (current_shader) {
              current_shader->use(ctx);
              ctx.render_context->apply();
            }
          }

          if (current_shader && geometries[i]) {

            UniformValue model_mat(skel_anim_node->get_cached_world_transform());
            UniformValue normal_mat(scm::math::transpose(scm::math::inverse(skel_anim_node->get_cached_world_transform())));

            current_shader->apply_uniform(ctx, "gua_model_matrix", model_mat);
            current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

            materials[i]->apply_uniforms(ctx, current_shader, view_id);

            ctx.render_context->apply_program();

            bones_block_.update(ctx.render_context, skel_anim_node->get_director()->get_bone_transforms());

            ctx.render_context->bind_uniform_buffer( bones_block_.block().block_buffer(), 1);
            geometries[i]->draw(ctx);
          }
        }
      }

      pipe.get_gbuffer().unbind(ctx);
    }
  }
} // namespace gua
