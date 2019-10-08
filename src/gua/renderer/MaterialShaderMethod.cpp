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
#include <gua/renderer/MaterialShaderMethod.hpp>

// guacamole headers
#include <gua/utils/TextFile.hpp>
#include <gua/utils/Logger.hpp>

#include <jsoncpp/json/json.h>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
MaterialShaderMethod::MaterialShaderMethod(std::string const& name) : name_(name) {}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderMethod& MaterialShaderMethod::load_from_file(std::string const& file_name)
{
    if(file_name != "")
    {
        TextFile file(file_name);

        if(file.is_valid())
        {
            load_from_json(file.get_content());
        }
        else
        {
            Logger::LOG_WARNING << "Failed to load material method \"" << file_name
                                << "\": "
                                   "File does not exist!"
                                << std::endl;
        }
    }

    return *this;
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderMethod& MaterialShaderMethod::load_from_json(std::string const& json_string)
{
    Json::Value value;
    Json::Reader reader;
    if(!reader.parse(json_string, value))
    {
        Logger::LOG_WARNING << "Failed to parse material method description: "
                               "Invalid json String!"
                            << std::endl;
        return *this;
    }

    if(value["name"] != Json::Value::null && value["source"] != Json::Value::null)
    {
        set_name(value["name"].asString());
        set_source(value["source"].asString());

        if(value["uniforms"] != Json::Value::null && value["uniforms"].isArray())
        {
            for(unsigned int i = 0; i < value["uniforms"].size(); ++i)
            {
                auto uniform_string(value["uniforms"][i]);
                if(uniform_string["name"] != Json::Value::null && uniform_string["type"] != Json::Value::null && uniform_string["value"] != Json::Value::null)
                {
                    auto uniform(UniformValue::create_from_strings(uniform_string["value"].asString(), uniform_string["type"].asString()));

                    set_uniform(uniform_string["name"].asString(), ViewDependentUniform(uniform));
                }
                else
                {
                    Logger::LOG_WARNING << "Failed to load uniform: "
                                           "Please provide name, type and value in the description!"
                                        << std::endl;
                }
            }
        }
    }
    else
    {
        Logger::LOG_WARNING << "Failed to load material method: "
                               "Please provide name and source in the description!"
                            << std::endl;
    }
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
MaterialShaderMethod& MaterialShaderMethod::set_name(std::string const& name)
{
    name_ = name;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& MaterialShaderMethod::get_name() const { return name_; }

////////////////////////////////////////////////////////////////////////////////
MaterialShaderMethod& MaterialShaderMethod::set_source(std::string const& source)
{
    source_ = source;
    return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& MaterialShaderMethod::get_source() const { return source_; }

////////////////////////////////////////////////////////////////////////////////

std::map<std::string, ViewDependentUniform> const& MaterialShaderMethod::get_uniforms() const { return uniforms_; }

////////////////////////////////////////////////////////////////////////////////

std::ostream& MaterialShaderMethod::serialize_uniforms_to_stream(std::ostream& os) const
{
    for(auto& uniform : uniforms_)
    {
        os << uniform.first << "#";
        uniform.second.serialize_to_stream(os);
        os << ";";
    }

    return os;
}

////////////////////////////////////////////////////////////////////////////////

void MaterialShaderMethod::set_uniforms_from_serialized_string(std::string const& value)
{
    auto tokens(string_utils::split(value, ';'));

    for(auto& token : tokens)
    {
        auto parts(string_utils::split(token, '#'));
        set_uniform(parts[0], ViewDependentUniform::create_from_serialized_string(parts[1]));
    }
}

} // namespace gua
