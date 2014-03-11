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
#include <gua/renderer/MaterialDescription.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils.hpp>
#include <gua/databases.hpp>

// external headers
#include <fstream>
#include <sstream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

MaterialDescription::MaterialDescription() : file_name_(""), shading_model_(), uniforms_() {}

////////////////////////////////////////////////////////////////////////////////

MaterialDescription::MaterialDescription(std::string const& file_name)
    : file_name_(file_name), shading_model_(), uniforms_() {

    reload();
}

////////////////////////////////////////////////////////////////////////////////

MaterialDescription::MaterialDescription(const char* buffer, unsigned buffer_size)
    : shading_model_(), uniforms_() {

    construct_from_buffer(buffer, buffer_size);
}

////////////////////////////////////////////////////////////////////////////////

MaterialDescription::~MaterialDescription() {}

////////////////////////////////////////////////////////////////////////////////

void MaterialDescription::reload() {
    if (file_name_ != "") {
        TextFile file(file_name_);

        if (file.is_valid()) {
            construct_from_file(file);
        } else {
            WARNING("Failed to load material \"%s\": "
                    "File does not exist!",
                    file_name_.c_str());
        }

        PathParser p;
        p.parse(file_name_);
        shading_model_ = p.get_path(true) + shading_model_;
    }
}

////////////////////////////////////////////////////////////////////////////////

void MaterialDescription::save_to_file(std::string const& file_name) const {
    Json::Value uniforms;
    for (auto uniform : uniforms_) {
        uniforms[uniform.first] = uniform.second;
    }

    Json::Value root;
    root["uniforms"] = uniforms;
    root["shading_model"] = shading_model_;

    std::stringstream str;
    str << root;

    TextFile file(file_name);
    file.set_content(str.str());
    file.save();
}

////////////////////////////////////////////////////////////////////////////////

void MaterialDescription::construct_from_file(TextFile const & file) {
    Json::Value value;
    Json::Reader reader;
    if (!reader.parse(file.get_content(), value)) {
        WARNING("Failed to parse material description \"%s\": "
                "File does not exist!",
                file.get_file_name().c_str());
        return;
    }

    if (value["shading_model"] == Json::Value::null) {
        WARNING("Failed to parse material description \"%s\": "
                "No shading model associated",
                file.get_file_name().c_str());
        return;
    }

    shading_model_ = value["shading_model"].asString();
    uniforms_.clear();

    if (value["uniforms"] != Json::Value::null) {
        auto uniforms_strings(value["uniforms"].getMemberNames());
        for (auto uniform : uniforms_strings) {
            uniforms_.insert(
                std::make_pair(uniform, value["uniforms"][uniform].asString()));
        }
    }

}

////////////////////////////////////////////////////////////////////////////////

void MaterialDescription::construct_from_buffer(const char* buffer, unsigned buffer_size) {
    Json::Value value;
    Json::Reader reader;
    if (!reader.parse(buffer, buffer + buffer_size, value)) {
        WARNING("Failed to parse material description from buffer: "
                "Buffer invalid!");
        return;
      }

    if (value["shading_model"] == Json::Value::null) {
        WARNING("Failed to parse material description from buffer: "
                "No shading model associated!");
        return;
    }

    shading_model_ = value["shading_model"].asString();

    if (value["uniforms"] != Json::Value::null) {
        auto uniforms_strings(value["uniforms"].getMemberNames());
        for (auto uniform : uniforms_strings) {
            uniforms_.insert(
                std::make_pair(uniform, value["uniforms"][uniform].asString()));
        }
    }

}

////////////////////////////////////////////////////////////////////////////////

}
