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
#include <gua/renderer/ShaderStage.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ShaderStage::ShaderStage()
    : uniforms_(), outputs_(), functions_(), body_() {}

////////////////////////////////////////////////////////////////////////////////

ShaderStage::ShaderStage(Json::Value const& value)
    : uniforms_(), outputs_(), functions_(), body_() {

    construct_from_json_string(value);
}

////////////////////////////////////////////////////////////////////////////////

Json::Value const ShaderStage::to_json_string() const {
    Json::Value root_value;

    root_value["uniforms"] = uniforms_to_json(uniforms_);
    root_value["outputs"] = outputs_to_json(outputs_);
    root_value["functions"] = functions_;
    root_value["body"] = body_;
    return root_value;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, UniformType>& ShaderStage::get_uniforms() {

    return uniforms_;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, BufferComponent>& ShaderStage::get_outputs() {

    return outputs_;
}

////////////////////////////////////////////////////////////////////////////////

std::string& ShaderStage::get_functions() {

    return functions_;
}

////////////////////////////////////////////////////////////////////////////////

std::string& ShaderStage::get_body() {

    return body_;
}

////////////////////////////////////////////////////////////////////////////////

void ShaderStage::construct_from_json_string(Json::Value const& value) {

    if (value["uniforms"] != Json::Value::null) {
        uniforms_ = json_to_uniform(value["uniforms"]);
    }

    if (value["outputs"] != Json::Value::null) {
        outputs_ = json_to_outputs(value["outputs"]);
    }

    if (value["functions"] != Json::Value::null) {
        functions_ = value["functions"].asString();
    }

    if (value["body"] != Json::Value::null) {
        body_ = value["body"].asString();
    }
}

////////////////////////////////////////////////////////////////////////////////

Json::Value const ShaderStage::uniforms_to_json(
    std::unordered_map<std::string, UniformType> const& uniforms) const {
    Json::Value uniforms_value;

    for (auto const& uniform : uniforms) {
        uniforms_value[uniform.first] =
            enums::uniform_type_to_string(uniform.second);
    }

    return uniforms_value;
}

////////////////////////////////////////////////////////////////////////////////

Json::Value const ShaderStage::set_to_json(std::set<std::string> const &
                                           set) const {
    Json::Value value;

    for (auto const& mem : set) {
        value.append(mem);
    }

    return value;
}

////////////////////////////////////////////////////////////////////////////////

Json::Value const ShaderStage::outputs_to_json(
    std::unordered_map<std::string, BufferComponent> const & outputs) const {
    Json::Value outputs_value;

    for (auto const& output : outputs) {
        outputs_value[output.first] =
            enums::output_type_to_string(output.second);
    }

    return outputs_value;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, UniformType>
ShaderStage::json_to_uniform(Json::Value const & value) {
    std::unordered_map<std::string, UniformType> uniforms;

    auto uniforms_strings(value.getMemberNames());

    for (auto const& uniform : uniforms_strings) {
        auto t = enums::parse_uniform_type(value[uniform].asString());
        if (t) {
          uniforms.insert(std::make_pair(uniform, *t));
        }
    }
    return uniforms;
}

////////////////////////////////////////////////////////////////////////////////

std::set<std::string> ShaderStage::json_to_set(Json::Value const & value) {
    std::set<std::string> set;
    if (!value.isArray()) {
        WARNING("Unable to parse Element: %s doesn't contain an array!",
                value.asCString());
        return set;
    }

    for (unsigned i(0); i < value.size(); ++i) {
        set.insert(value[i].asString());
    }
    return set;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, BufferComponent>
ShaderStage::json_to_outputs(Json::Value const & value) {

    auto outputs_strings(value.getMemberNames());

    std::unordered_map<std::string, BufferComponent> outputs;
    for (auto const& output : outputs_strings) {
        auto t = enums::parse_output_type(value[output].asString());
        if (t) {
          outputs.insert(std::make_pair(output, *t));
        }
    }
    return outputs;
}

////////////////////////////////////////////////////////////////////////////////

}
