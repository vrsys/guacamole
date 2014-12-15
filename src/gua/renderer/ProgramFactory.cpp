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

#include <gua/renderer/ProgramFactory.hpp>

#include <fstream>

#include <gua/config.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/MaterialShader.hpp>


namespace gua {

  ///////////////////////////////////////////////////////////////////////////
  ProgramFactory::ProgramFactory(std::vector<std::string> const& shader_root_directories)
    : _search_paths(shader_root_directories)
  {
    add_search_path(std::string(GUACAMOLE_INSTALL_DIR));
    add_search_path(std::string(GUACAMOLE_INSTALL_DIR) + "/resources/");
  }

  ///////////////////////////////////////////////////////////////////////////
  ProgramFactory::~ProgramFactory()
  {}

  ///////////////////////////////////////////////////////////////////////////
  void ProgramFactory::add_search_path(std::string const& path)
  {
    _search_paths.push_back(path);
  }

  ///////////////////////////////////////////////////////////////////////////
  std::shared_ptr<ShaderProgram> ProgramFactory::create_program(MaterialShader* material,
                                                                std::map<scm::gl::shader_stage, std::string> const& program_description,
                                                                std::list<std::string> const& interleaved_stream_capture,
                                                                bool in_rasterization_discard)
  {
    // std::type_index type_id(typeid(for_type));
    using namespace scm::gl;

    std::vector<ShaderProgramStage> final_program_description;
    auto new_shader = std::make_shared<ShaderProgram>();

    auto v_methods = material->get_vertex_methods();
    auto f_methods = material->get_fragment_methods();

    for (auto const& stage : program_description)
    {
      // insert material code in vertex and fragment shader
      if (stage.first == STAGE_VERTEX_SHADER) {
        auto v_shader(compile_description(material, v_methods, program_description.at(STAGE_VERTEX_SHADER)));
        final_program_description.push_back(ShaderProgramStage(STAGE_VERTEX_SHADER, v_shader));
      }
      else {
        if (stage.first == STAGE_GEOMETRY_SHADER) {
          auto g_shader(compile_description(material, v_methods, program_description.at(STAGE_GEOMETRY_SHADER)));
          final_program_description.push_back(ShaderProgramStage(STAGE_GEOMETRY_SHADER, g_shader));
        }
        else {
          if (stage.first == STAGE_FRAGMENT_SHADER) {
            auto f_shader(compile_description(material, f_methods, program_description.at(STAGE_FRAGMENT_SHADER)));
            final_program_description.push_back(ShaderProgramStage(STAGE_FRAGMENT_SHADER, f_shader));
          }
          else {
            // keep code for other shading stages
            final_program_description.push_back(ShaderProgramStage(stage.first, stage.second));
          }
        }
      }
    }

    new_shader->set_shaders(final_program_description, interleaved_stream_capture, in_rasterization_discard);
    return new_shader;
  }


  ////////////////////////////////////////////////////////////////////////////////
  std::string ProgramFactory::compile_description(MaterialShader* material,
                                                   std::list<MaterialShaderMethod> const& methods,
                                                   std::string const& shader_source) const 
  {
    std::string source(shader_source);
    std::stringstream sstr;

    for (auto const& uniform : material->get_default_material()->get_uniforms()) {
      sstr << "uniform " << uniform.second.get().get_glsl_type() << " "
        << uniform.first << ";" << std::endl;
    }
    sstr << std::endl;

    // insert uniforms
    gua::string_utils::replace(source, "@material_uniforms", sstr.str());
    gua::string_utils::replace(source, "@material_input", "");

    sstr.str("");

    // material methods ----------------------------------------------------------
    for (auto& method : methods) {
      sstr << method.get_source() << std::endl;
    }
    gua::string_utils::replace(source, "@material_method_declarations", sstr.str());
    sstr.str("");

    // material method calls -----------------------------------------------------
    for (auto& method : methods) {
      sstr << method.get_name() << "();" << std::endl;
    }
    gua::string_utils::replace(source, "@material_method_calls", sstr.str());

    // std::cout << string_utils::format_code(source) << std::endl;

    // indent and return code ----------------------------------------------------
    return string_utils::format_code(source);
  }


  ///////////////////////////////////////////////////////////////////////////
  std::string ProgramFactory::read_shader_from_file(std::string const& path) const
  {
    std::cout << "readng : " << path << std::endl;
    try {
      std::string full_path(path);
      std::ifstream ifstr(full_path.c_str(), std::ios::in);

      if (ifstr.good()) {
        return std::string(std::istreambuf_iterator<char>(ifstr), std::istreambuf_iterator<char>());
      }

      for (auto const& root : _search_paths)
      {
        std::string full_path(root + std::string("/") + path);
        std::ifstream ifstr(full_path.c_str(), std::ios::in);

        if (ifstr.good()) {
          auto source = std::string(std::istreambuf_iterator<char>(ifstr), std::istreambuf_iterator<char>());
          resolve_shader_includes(source);
          return source;
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
  void ProgramFactory::resolve_shader_includes(std::string& shader_source) const
  {
    std::size_t search_pos(0);

    std::string search("@include");

    while (search_pos != std::string::npos)
    {
      search_pos = shader_source.find(search, search_pos);

      if (search_pos != std::string::npos) {

        std::size_t start(shader_source.find('\"', search_pos) + 1);
        std::size_t end(shader_source.find('\"', start));

        std::string file(shader_source.substr(start, end - start));

        std::string include = read_shader_from_file(file);
        shader_source.replace(search_pos, end - search_pos + 2, include);

        // advance search pos
        search_pos = search_pos + include.length();
      }
    }
  }

  
}
