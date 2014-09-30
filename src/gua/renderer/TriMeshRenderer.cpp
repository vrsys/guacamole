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

#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>

#define MATERIAL_BUFFER_SIZE 1024*128

namespace gua {

////////////////////////////////////////////////////////////////////////////////

TriMeshRenderer::TriMeshRenderer() :
  vertex_shader_(
   Resources::lookup_shader(Resources::shaders_tri_mesh_shader_vert)
 ),
 fragment_shader_(
   Resources::lookup_shader(Resources::shaders_tri_mesh_shader_frag)
 ),
 material_uniform_storage_buffer_(nullptr) {}

////////////////////////////////////////////////////////////////////////////////

void TriMeshRenderer::draw(std::unordered_map<std::string, std::vector<node::GeometryNode*>> const& sorted_objects,
                           Pipeline* pipe) const {

  auto const& ctx(pipe->get_context());

  // initialize storage buffer
  if (!material_uniform_storage_buffer_) {
    material_uniform_storage_buffer_ = ctx.render_device->create_buffer(
      scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STATIC_READ, MATERIAL_BUFFER_SIZE);
  }

  // loop through all materials ------------------------------------------------
  for (auto const& object_list : sorted_objects) {

    auto const& material = MaterialDatabase::instance()->lookup(object_list.first);
    if (material) {
      unsigned object_count(object_list.second.size());

      /*
      char* buffer = reinterpret_cast<char*>(ctx.render_context->map_buffer(material_uniform_storage_buffer_, scm::gl::ACCESS_WRITE_ONLY));
      
      unsigned current_pos(0);
      
      // gather all uniform of all nodes for this material ---------------------
      for (int i(0); i<object_count; i++) {
        auto const& node(object_list.second[i]);
        
        UniformValue model_mat("gua_model_matrix", node->get_cached_world_transform());
        UniformValue normal_mat("gua_normal_matrix", scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())));

        current_pos += model_mat.write_bytes(ctx, buffer + current_pos);
        current_pos += normal_mat.write_bytes(ctx, buffer + current_pos);

        // for (auto const& uniform : material->get_default_instance().get_uniforms()) {
          // UniformValue const* value(&uniform);
          for (auto const& overwrite : node->get_material().get_uniforms()) {
            // if (overwrite.get_name() == uniform.get_name()) {
              // value = &overwrite;    
            current_pos += overwrite.write_bytes(ctx, buffer + current_pos);
              // break;
            // }
          }
          // current_pos += value->write_bytes(ctx, buffer + current_pos);
        // }

        int padding(sizeof(math::vec4) - current_pos % sizeof(math::vec4));
        current_pos += padding % sizeof(math::vec4);
      }

      ctx.render_context->unmap_buffer(material_uniform_storage_buffer_);
      ctx.render_context->bind_storage_buffer(material_uniform_storage_buffer_, 0);
      */

      // get shader for this material
      auto const& ressource = GeometryDatabase::instance()->lookup(object_list.second[0]->get_filename());
      auto const& shader(material->get_shader(*ressource, vertex_shader_, fragment_shader_));
      shader->use(ctx);

      // draw all objects ------------------------------------------------------
      for (int i(0); i<object_count; i++) {
        auto const& node(object_list.second[i]);

        auto const& ressource = GeometryDatabase::instance()->lookup(node->get_filename());
        if (ressource) {

          ///*
          material->apply_uniforms(ctx, shader, node->get_material());
          shader->set_uniform(ctx, node->get_cached_world_transform(), "gua_model_matrix");
          shader->set_uniform(ctx, scm::math::transpose(scm::math::inverse(node->get_cached_world_transform())), "gua_normal_matrix");
          //*/

          //shader->set_uniform(ctx, i, "gua_draw_index");
          ressource->draw(ctx);

        } else {
          Logger::LOG_WARNING << "GBufferPass::process(): Cannot find geometry ressource: " << node->get_filename() << std::endl;
        }

      }

      ctx.render_context->reset_storage_buffers();

    } else {
      Logger::LOG_WARNING << "GBufferPass::process(): Cannot find material: " << object_list.first << std::endl;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

}

