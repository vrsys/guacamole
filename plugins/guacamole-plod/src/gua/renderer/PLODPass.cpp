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
#include <gua/renderer/PLODPass.hpp>

// guacamole headers
#include <gua/renderer/PLODResource.hpp>
#include <gua/renderer/PLODRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/config.hpp>

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

////////////////////////////////////////////////////////////////////////////////

PLODPassDescription::PLODPassDescription()
  : PipelinePassDescription()
{
  needs_color_buffer_as_input_ = false;
  writes_only_color_buffer_ = false;
  doClear_ = false;
  rendermode_ = RenderMode::Custom;
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* PLODPassDescription::make_copy() const {
  return new PLODPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass PLODPassDescription::make_pass(RenderContext const& ctx)
{
  PipelinePass pass{ *this, ctx };

  auto renderer = std::make_shared<PLODRenderer>();

  pass.process_ = [renderer](
    PipelinePass&, PipelinePassDescription const&, Pipeline & pipe) {
    renderer->render(pipe);
  };

  return pass;
}

}
