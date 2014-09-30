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

      if (value["vertex_passes"] != Json::Value::null
          && value["vertex_passes"].isArray()) {

        for (int i(0); i < value["vertex_passes"].size(); ++i) {
          auto pass(value["vertex_passes"][i]);
          MaterialPass vertex_pass;

          // load pass from file if file name is set
          if (pass["file_name"] != Json::Value::null) {
            vertex_pass.load_from_file(pass["file_name"].asString());
          // else use name and source
          } else {
            std::cout << pass << std::endl;
            vertex_pass.load_from_json(pass.toStyledString());
          }

          add_vertex_pass(vertex_pass);
        }
      }

      if (value["fragment_passes"] != Json::Value::null
          && value["fragment_passes"].isArray()) {

        for (int i(0); i < value["fragment_passes"].size(); ++i) {
          auto pass(value["vertex_passes"][i]);
          MaterialPass fragment_pass;

          // load pass from file if file name is set
          if (pass["file_name"] != Json::Value::null) {
            fragment_pass.load_from_file(pass["file_name"].asString());
          // else use name and source
          } else {
            fragment_pass.load_from_json(pass.toStyledString());
          }

          add_fragment_pass(fragment_pass);
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
MaterialShaderDescription& MaterialShaderDescription::add_vertex_pass(MaterialPass const& pass) {
  vertex_passes_.push_back(pass);
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderDescription& MaterialShaderDescription::add_fragment_pass(MaterialPass const& pass) {
  fragment_passes_.push_back(pass);
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::list<MaterialPass> const& MaterialShaderDescription::get_vertex_passes() const {
  return vertex_passes_;
}

////////////////////////////////////////////////////////////////////////////////
std::list<MaterialPass> const& MaterialShaderDescription::get_fragment_passes() const {
  return fragment_passes_;
}

}
