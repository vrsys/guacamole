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

  namespace {

    ///////////////////////////////////////////////////////////////////////////
    std::string read_shader_file(std::string const& path, std::vector<std::string> const& root_dirs)
    {
      try {
        std::string full_path(path);
        std::ifstream ifstr(full_path.c_str(), std::ios::in);

        if (ifstr.good()) {
          return std::string(std::istreambuf_iterator<char>(ifstr), std::istreambuf_iterator<char>());
        }

        for (auto const& root : root_dirs)
        {
          std::string full_path(root + std::string("/") + path);
          std::ifstream ifstr(full_path.c_str(), std::ios::in);

          if (ifstr.good()) {
            return std::string(std::istreambuf_iterator<char>(ifstr), std::istreambuf_iterator<char>());
          }
          ifstr.close();
        }
        throw std::runtime_error("File not found.");
      }
      catch (...) {
        std::cerr << "Error reading file : " << path << std::endl;
        return "";
      }
    }

    ///////////////////////////////////////////////////////////////////////////
    void resolve_includes(std::string& shader_source, std::vector<std::string> const& root_dirs)
    {
      std::size_t search_pos(0);

      std::string search("#include");

      while (search_pos != std::string::npos) 
      {
        search_pos = shader_source.find(search, search_pos);

        if (search_pos != std::string::npos) {

          std::size_t start(shader_source.find('\"', search_pos) + 1);
          std::size_t end(shader_source.find('\"', start));

          std::string file(shader_source.substr(start, end - start));

          std::string include = read_shader_file(file, root_dirs);
          shader_source.replace(search_pos, end - search_pos + 2, include);

          // advance search pos
          search_pos = search_pos + include.length();
        }
      }
    }
  }


#if 0

  TriMeshPassDescription::TriMeshPassDescription()
    : PipelinePassDescription() {
    vertex_shader_ = ""; // "shaders/tri_mesh_shader.vert";
    fragment_shader_ = ""; // "shaders/tri_mesh_shader.frag";

    needs_color_buffer_as_input_ = false;
    writes_only_color_buffer_ = false;
    doClear_ = false;
    rendermode_ = RenderMode::Custom;

    std::shared_ptr<scm::gl::buffer_ptr> material_uniform_storage_buffer = std::make_shared<scm::gl::buffer_ptr>(nullptr);
    auto vertex_shader = Resources::lookup_shader("shaders/tri_mesh_shader.vert");
    auto fragment_shader = Resources::lookup_shader("shaders/tri_mesh_shader.frag");
    process_ = [material_uniform_storage_buffer, vertex_shader, fragment_shader](
      PipelinePass&, PipelinePassDescription const&, Pipeline & pipe) {

      auto sorted_objects(pipe.get_scene().nodes.find(std::type_index(typeid(node::TriMeshNode))));

      if (sorted_objects != pipe.get_scene().nodes.end() && sorted_objects->second.size() > 0) {

        std::sort(sorted_objects->second.begin(), sorted_objects->second.end(), [](node::Node* a, node::Node* b){
          return reinterpret_cast<node::TriMeshNode*>(a)->get_material().get_shader() < reinterpret_cast<node::TriMeshNode*>(b)->get_material().get_shader();
        });

        RenderContext const& ctx(pipe.get_context());

        bool writes_only_color_buffer = false;
        pipe.get_gbuffer().bind(ctx, writes_only_color_buffer);
        pipe.get_gbuffer().set_viewport(ctx);
        int view_id(pipe.get_camera().config.get_view_id());

        MaterialShader* current_material(nullptr);
        ShaderProgram*  current_shader(nullptr);

        // loop through all objects, sorted by material ----------------------------
        for (auto const& object : sorted_objects->second) {

          auto tri_mesh_node(reinterpret_cast<node::TriMeshNode*>(object));

          if (current_material != tri_mesh_node->get_material().get_shader()) {
            current_material = tri_mesh_node->get_material().get_shader();
            if (current_material) {
              current_shader = current_material->get_shader(ctx, typeid(*tri_mesh_node->get_geometry()), vertex_shader, fragment_shader);
            }
            else {
              Logger::LOG_WARNING << "TriMeshPass::process(): Cannot find material: " << tri_mesh_node->get_material().get_shader_name() << std::endl;
            }
            if (current_shader) {
              current_shader->use(ctx);
              ctx.render_context->apply();
            }
          }

          if (current_shader && tri_mesh_node->get_geometry()) {
            UniformValue model_mat(tri_mesh_node->get_cached_world_transform());
            UniformValue normal_mat(scm::math::transpose(scm::math::inverse(tri_mesh_node->get_cached_world_transform())));

            current_shader->apply_uniform(ctx, "gua_model_matrix", model_mat);
            current_shader->apply_uniform(ctx, "gua_normal_matrix", normal_mat);

            for (auto const& overwrite : tri_mesh_node->get_material().get_uniforms()) {
              current_shader->apply_uniform(ctx, overwrite.first, overwrite.second.get(view_id));
            }

            ctx.render_context->apply_program();

            tri_mesh_node->get_geometry()->draw(ctx);
          }
        }

        pipe.get_gbuffer().unbind(ctx);
      }
    };
  }

  ////////////////////////////////////////////////////////////////////////////////

  PipelinePassDescription* TriMeshPassDescription::make_copy() const {
    return new TriMeshPassDescription(*this);
  }


#endif

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

PipelinePass NURBSPassDescription::make_pass(RenderContext const& ctx)
{
  PipelinePass pass{ *this, ctx };

  auto renderer = std::make_shared<NURBSRenderer>();

  pass.process_ = [renderer](
    PipelinePass&, PipelinePassDescription const&, Pipeline & pipe) {
    renderer->render(pipe);
  };

  return pass;
}

}
