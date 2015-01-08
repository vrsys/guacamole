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

#ifndef GUA_PROGRAM_FACTORY_HPP
#define GUA_PROGRAM_FACTORY_HPP

#include <vector>
#include <map>
#include <unordered_map>
#include <list>

#include <scm/gl_core/shader_objects.h>

#include <gua/platform.hpp>
#include <boost/filesystem.hpp>


namespace gua {

struct RenderContext;
class ShaderProgram;
class MaterialShader;
class MaterialShaderMethod;

class GUA_DLL ProgramFactory {
 public:

  typedef std::unordered_map<std::string, std::string> SubstitutionMap;

  ProgramFactory(std::vector<std::string> const& search_directories = std::vector<std::string>());

  virtual ~ProgramFactory() {}

  void         add_search_path(std::string const& path);

  std::shared_ptr<ShaderProgram>  create_program(MaterialShader* material,
                                                 std::map<scm::gl::shader_stage, std::string> const& program_description,
                                                 std::list<std::string> const& interleaved_stream_capture = std::list<std::string>(),
                                                 bool in_rasterization_discard = false);

  std::string  read_plain_file(std::string const& file) const;
  std::string  read_shader_file(std::string const& file) const;
  std::string  compile_description(MaterialShader* material,
                                   std::list<MaterialShaderMethod> const& passes,
                                   std::string const& shader_source) const;
  std::string  resolve_substitutions(std::string const& shader_source,
                                     SubstitutionMap const& smap) const;

 private:

  bool         get_file_contents(boost::filesystem::path const& filename,
                                 boost::filesystem::path const& current_dir,
                                 std::string& contents,
                                 boost::filesystem::path& full_path) const;
  bool         resolve_includes(boost::filesystem::path const& filename,
                                boost::filesystem::path const& current_dir,
                                std::string& contents) const;

  std::vector<std::string> _search_paths;

};

}

#endif  // GUA_PIPELINE_HPP
