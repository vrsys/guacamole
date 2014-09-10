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
#include <gua/renderer/ShadingModel.hpp>

// guacamole headers
#include <gua/utils/Logger.hpp>
#include <gua/utils/TextFile.hpp>

// external headers
#include <fstream>
#include <sstream>

namespace gua {

unsigned ShadingModel::current_revision = 0;

////////////////////////////////////////////////////////////////////////////////

ShadingModel::ShadingModel()
    : file_name_(""), shader_stages_(4), name_("Unnamed shading model") {

   ++current_revision;
}

////////////////////////////////////////////////////////////////////////////////

ShadingModel::ShadingModel(std::string const& name)
    : file_name_(""), shader_stages_(4), name_(name) {

  ++current_revision;
}

////////////////////////////////////////////////////////////////////////////////

ShadingModel::ShadingModel(std::string const& name,
                           std::string const& file_name)
    : file_name_(file_name), shader_stages_(4), name_(name) {

  reload();
}

////////////////////////////////////////////////////////////////////////////////

void ShadingModel::reload() {
  ++current_revision;

  if (file_name_ != "") {
    TextFile file(file_name_);

    if (file.is_valid()) {
      construct_from_file(file);
    } else {
      Logger::LOG_WARNING << "Failed to load shading model \"" << file_name_ << "\": "
              "File does not exist!" << std::endl;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

ShadingModel::ShadingModel(std::string const& name,
                           const char* buffer,
                           unsigned buffer_size)
    : shader_stages_(4), name_(name) {

  construct_from_buffer(buffer, buffer_size);
}

ShaderStage& ShadingModel::get_gbuffer_vertex_stage() {

  return shader_stages_[GBUFFER_VERTEX_STAGE];
}

////////////////////////////////////////////////////////////////////////////////

ShaderStage& ShadingModel::get_gbuffer_fragment_stage() {

  return shader_stages_[GBUFFER_FRAGMENT_STAGE];
}

////////////////////////////////////////////////////////////////////////////////

ShaderStage& ShadingModel::get_lbuffer_stage() {

  return shader_stages_[LIGHTING_STAGE];
}

////////////////////////////////////////////////////////////////////////////////

ShaderStage& ShadingModel::get_final_shading_stage() {

  return shader_stages_[FINAL_STAGE];
}

////////////////////////////////////////////////////////////////////////////////

void ShadingModel::save_to_file(std::string const& file_name) const {

  std::stringstream str;

  Json::Value root;
  root["gbuffer_vertex_stage"] =
      shader_stages_[GBUFFER_VERTEX_STAGE].to_json_string();
  root["gbuffer_fragment_stage"] =
      shader_stages_[GBUFFER_FRAGMENT_STAGE].to_json_string();
  root["lbuffer_stage"] = shader_stages_[LIGHTING_STAGE].to_json_string();
  root["final_shading_stage"] = shader_stages_[FINAL_STAGE].to_json_string();

  str << root;

  TextFile file(file_name);
  file.set_content(str.str());
  file.save();
}

void ShadingModel::construct_from_file(TextFile const& file) {
  shader_stages_ = std::vector<ShaderStage>(4);

  Json::Value value;
  Json::Reader reader;
  if (!reader.parse(file.get_content(), value)) {
    Logger::LOG_WARNING << "Failed to parse shading model \"" << file.get_file_name() << "\": "
            "File does not exist!" << std::endl;
    return;
  }

  if (value["gbuffer_vertex_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model \"" << file.get_file_name() << "\": "
            "Gbuffer-Vertex-Stage is missing!" << std::endl;
    return;
  }

  if (value["gbuffer_fragment_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model \"" << file.get_file_name() << "\": "
            "Gbuffer-Fragment-Stage is missing!" << std::endl;
    return;
  }

  if (value["lbuffer_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model \"" << file.get_file_name() << "\": "
            "Lbuffer-stage is missing!" << std::endl;
    return;
  }

  if (value["final_shading_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model \"" << file.get_file_name() << "\": "
            "Final-Shading-Stage is missing!" << std::endl;
    return;
  }

  shader_stages_[GBUFFER_VERTEX_STAGE] = ShaderStage(value["gbuffer_vertex_stage"]);
  shader_stages_[GBUFFER_FRAGMENT_STAGE] = ShaderStage(value["gbuffer_fragment_stage"]);
  shader_stages_[LIGHTING_STAGE] = ShaderStage(value["lbuffer_stage"]);
  shader_stages_[FINAL_STAGE] = ShaderStage(value["final_shading_stage"]);

  // add default layers
  shader_stages_[GBUFFER_VERTEX_STAGE].get_outputs()["gua_position"] = BufferComponent::F3;
  shader_stages_[GBUFFER_FRAGMENT_STAGE].get_outputs()["gua_normal"] = BufferComponent::F3;
  shader_stages_[FINAL_STAGE].get_outputs()["gua_color"] = BufferComponent::F3;
}

void ShadingModel::construct_from_buffer(const char* buffer, unsigned buffer_size) {
  Json::Value value;
  Json::Reader reader;
  if (!reader.parse(buffer, buffer + buffer_size, value)) {
    Logger::LOG_WARNING << "Failed to parse shading model from buffer: "
            "Buffer invalid!" << std::endl;
    return;
  }

  if (value["gbuffer_vertex_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model from buffer: "
            "Gbuffer-Vertex-Stage is missing!" << std::endl;
    return;
  }

  if (value["gbuffer_fragment_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model from buffer: "
            "Gbuffer-Fragment-Stage is missing!" << std::endl;
    return;
  }

  if (value["lbuffer_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model from buffer: "
            "Lbuffer-stage is missing!" << std::endl;
    return;
  }

  if (value["final_shading_stage"] == Json::Value::null) {
    Logger::LOG_WARNING << "Failed to parse shading model from buffer: "
            "Final-Shading-Stage is missing!" << std::endl;
    return;
  }

  shader_stages_[GBUFFER_VERTEX_STAGE] = ShaderStage(value["gbuffer_vertex_stage"]);
  shader_stages_[GBUFFER_FRAGMENT_STAGE] = ShaderStage(value["gbuffer_fragment_stage"]);
  shader_stages_[LIGHTING_STAGE] = ShaderStage(value["lbuffer_stage"]);
  shader_stages_[FINAL_STAGE] = ShaderStage(value["final_shading_stage"]);

  // add default layers
  shader_stages_[GBUFFER_VERTEX_STAGE].get_outputs()["gua_position"] = BufferComponent::F3;
  shader_stages_[GBUFFER_FRAGMENT_STAGE].get_outputs()["gua_normal"] = BufferComponent::F3;
  shader_stages_[FINAL_STAGE].get_outputs()["gua_color"] = BufferComponent::F3;
}

}
