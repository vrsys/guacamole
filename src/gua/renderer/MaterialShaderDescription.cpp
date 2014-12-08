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

#include <gua/renderer/MaterialShaderDescription.hpp>

#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/renderer/Uniform.hpp>

#include <jsoncpp/json/json.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
void MaterialShaderDescription::load_from_file(std::string const& file_name) {
  if (file_name != "") {
    TextFile file(file_name);

    if (file.is_valid()) {
      Json::Value value;
      Json::Reader reader;
      if (!reader.parse(file.get_content(), value)) {
        Logger::LOG_WARNING << "Failed to parse material description \"" << file.get_file_name() << "\": "
                "File does not exist!" << std::endl;
        return;
      }

      file_name_ = file_name;
      vertex_methods_.clear();
      fragment_methods_.clear();

      if (value["vertex_methods"] != Json::Value::null
          && value["vertex_methods"].isArray()) {


        for (int i(0); i < value["vertex_methods"].size(); ++i) {
          auto method(value["vertex_methods"][i]);
          MaterialShaderMethod vertex_method;

          std::cout << "vertex method" << std::endl;
          std::cout << method << std::endl;
          // load method from file if file name is set
          if (method["file_name"] != Json::Value::null) {
            vertex_method.load_from_file(method["file_name"].asString());
          // else use name and source
          } else {
            vertex_method.load_from_json(method.toStyledString());
          }

          add_vertex_method(vertex_method);
        }
      }

      if (value["fragment_methods"] != Json::Value::null
          && value["fragment_methods"].isArray()) {


        for (int i(0); i < value["fragment_methods"].size(); ++i) {
          auto method(value["fragment_methods"][i]);
          MaterialShaderMethod fragment_method;

          // load method from file if file name is set
          if (method["file_name"] != Json::Value::null) {
            fragment_method.load_from_file(method["file_name"].asString());
          // else use name and source
          } else {
            fragment_method.load_from_json(method.toStyledString());
          }

          add_fragment_method(fragment_method);
        }
      }

    } else {
      Logger::LOG_WARNING << "Failed to load material description\""
                          << file_name << "\": "
                          "File does not exist!" << std::endl;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderDescription& MaterialShaderDescription::add_vertex_method(MaterialShaderMethod const& method) {
  vertex_methods_.push_back(method);
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderDescription& MaterialShaderDescription::add_fragment_method(MaterialShaderMethod const& method) {
  fragment_methods_.push_back(method);
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::list<MaterialShaderMethod> const& MaterialShaderDescription::get_vertex_methods() const {
  return vertex_methods_;
}

////////////////////////////////////////////////////////////////////////////////
std::list<MaterialShaderMethod> const& MaterialShaderDescription::get_fragment_methods() const {
  return fragment_methods_;
}

}
