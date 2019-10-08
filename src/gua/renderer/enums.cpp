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

#include <gua/renderer/enums.hpp>

#include <gua/math/math.hpp>
#include <gua/utils/string_utils.hpp>
#include <boost/optional.hpp>
#include <boost/none.hpp>

namespace gua
{
namespace enums
{
std::string output_type_to_string(BufferComponent type)
{
    switch(type)
    {
    case BufferComponent::I1:
        return "int";
    case BufferComponent::I2:
        return "ivec2";
    case BufferComponent::I3:
        return "ivec3";
    case BufferComponent::I4:
        return "ivec4";
    case BufferComponent::U1:
        return "uint";
    case BufferComponent::U2:
        return "uvec2";
    case BufferComponent::U3:
        return "uvec3";
    case BufferComponent::U4:
        return "uvec4";
    case BufferComponent::H1:
        return "half";
    case BufferComponent::H2:
        return "half2";
    case BufferComponent::H3:
        return "half3";
    case BufferComponent::H4:
        return "half4";
    case BufferComponent::F1:
        return "float";
    case BufferComponent::F2:
        return "vec2";
    case BufferComponent::F3:
        return "vec3";
    case BufferComponent::F4:
        return "vec4";

    case BufferComponent::DEPTH_16:
        return "depth16";
    case BufferComponent::DEPTH_24:
        return "depth24";

    default:
        return "undefined";
    }
}

////////////////////////////////////////////////////////////////////////////////

boost::optional<BufferComponent> parse_output_type(std::string const& type)
{
    if(type == "int")
        return boost::make_optional(BufferComponent::I1);
    if(type == "ivec2")
        return boost::make_optional(BufferComponent::I2);
    if(type == "ivec3")
        return boost::make_optional(BufferComponent::I3);
    if(type == "ivec4")
        return boost::make_optional(BufferComponent::I4);
    if(type == "uint")
        return boost::make_optional(BufferComponent::U1);
    if(type == "uvec2")
        return boost::make_optional(BufferComponent::U2);
    if(type == "uvec3")
        return boost::make_optional(BufferComponent::U3);
    if(type == "uvec4")
        return boost::make_optional(BufferComponent::U4);
    if(type == "half")
        return boost::make_optional(BufferComponent::H1);
    if(type == "half2")
        return boost::make_optional(BufferComponent::H2);
    if(type == "half3")
        return boost::make_optional(BufferComponent::H3);
    if(type == "half4")
        return boost::make_optional(BufferComponent::H4);
    if(type == "float")
        return boost::make_optional(BufferComponent::F1);
    if(type == "vec2")
        return boost::make_optional(BufferComponent::F2);
    if(type == "vec3")
        return boost::make_optional(BufferComponent::F3);
    if(type == "vec4")
        return boost::make_optional(BufferComponent::F4);

    if(type == "depth16")
        return boost::make_optional(BufferComponent::DEPTH_16);
    if(type == "depth24")
        return boost::make_optional(BufferComponent::DEPTH_24);

    return boost::none;
}

////////////////////////////////////////////////////////////////////////////////

std::string uniform_type_to_string(UniformType type)
{
    switch(type)
    {
    case UniformType::INT:
        return "int";
    case UniformType::FLOAT:
        return "float";
    case UniformType::BOOL:
        return "bool";
    case UniformType::VEC2:
        return "vec2";
    case UniformType::VEC3:
        return "vec3";
    case UniformType::VEC4:
        return "vec4";
    case UniformType::VEC2I:
        return "vec2i";
    case UniformType::VEC3I:
        return "vec3i";
    case UniformType::VEC4I:
        return "vec4i";
    case UniformType::VEC2UI:
        return "vec2ui";
    case UniformType::VEC3UI:
        return "vec3ui";
    case UniformType::VEC4UI:
        return "vec4ui";
    case UniformType::MAT3:
        return "mat3";
    case UniformType::MAT4:
        return "mat4";
    case UniformType::SAMPLER1D:
        return "sampler1D";
    case UniformType::SAMPLER2D:
        return "sampler2D";
    case UniformType::SAMPLER3D:
        return "sampler3D";
    case UniformType::SAMPLERCUBE:
        return "samplerCube";
    default:
        return "undefined";
    }
}

////////////////////////////////////////////////////////////////////////////////

boost::optional<UniformType> parse_uniform_type(std::string const& type)
{
    if(type == "int")
        return boost::make_optional(UniformType::INT);
    if(type == "float")
        return boost::make_optional(UniformType::FLOAT);
    if(type == "bool")
        return boost::make_optional(UniformType::BOOL);
    if(type == "vec2")
        return boost::make_optional(UniformType::VEC2);
    if(type == "vec3")
        return boost::make_optional(UniformType::VEC3);
    if(type == "vec4")
        return boost::make_optional(UniformType::VEC4);
    if(type == "vec2i")
        return boost::make_optional(UniformType::VEC2I);
    if(type == "vec3i")
        return boost::make_optional(UniformType::VEC3I);
    if(type == "vec4i")
        return boost::make_optional(UniformType::VEC4I);
    if(type == "vec2ui")
        return boost::make_optional(UniformType::VEC2UI);
    if(type == "vec3ui")
        return boost::make_optional(UniformType::VEC3UI);
    if(type == "vec4ui")
        return boost::make_optional(UniformType::VEC4UI);
    if(type == "mat3")
        return boost::make_optional(UniformType::MAT3);
    if(type == "mat4")
        return boost::make_optional(UniformType::MAT4);
    if(type == "sampler1D")
        return boost::make_optional(UniformType::SAMPLER1D);
    if(type == "sampler2D")
        return boost::make_optional(UniformType::SAMPLER2D);
    if(type == "sampler3D")
        return boost::make_optional(UniformType::SAMPLER3D);
    if(type == "samplerCube")
        return boost::make_optional(UniformType::SAMPLERCUBE);

    return boost::none;
}

std::string buffer_component_type_to_string(BufferComponentType type)
{
    switch(type)
    {
    case TYPE_INTEGER:
        return "int";
    case TYPE_UNSIGNED:
        return "uint";
    case TYPE_HALF:
        return "half";
    case TYPE_FLOAT:
        return "float";
    case TYPE_DEPTH:
        return "depth";
    default:
        return "undefined";
    }
}

boost::optional<BufferComponentType> parse_buffer_component_type(std::string const& type)
{
    if(type == "int")
        return boost::make_optional(TYPE_INTEGER);
    if(type == "uint")
        return boost::make_optional(TYPE_UNSIGNED);
    if(type == "half")
        return boost::make_optional(TYPE_HALF);
    if(type == "float")
        return boost::make_optional(TYPE_FLOAT);
    if(type == "depth")
        return boost::make_optional(TYPE_DEPTH);

    return boost::none;
}

////////////////////////////////////////////////////////////////////////////////

std::string get_default_value(UniformType type)
{
    switch(type)
    {
    case UniformType::INT:
        return string_utils::to_string(0);
    case UniformType::FLOAT:
        return string_utils::to_string(0.f);
    case UniformType::BOOL:
        return string_utils::to_string(false);
    case UniformType::VEC2:
        return string_utils::to_string(math::vec2::zero());
    case UniformType::VEC3:
        return string_utils::to_string(math::vec3::zero());
    case UniformType::VEC4:
        return string_utils::to_string(math::vec4::zero());
    case UniformType::MAT3:
    {
        auto s(string_utils::to_string(math::mat3::identity()));
        string_utils::replace(s, "\n", ";");
        return s;
    }
    case UniformType::MAT4:
    {
        auto s(string_utils::to_string(math::mat4::identity()));
        string_utils::replace(s, "\n", ";");
        return s;
    }
    case UniformType::SAMPLER1D:
        return "path/to/texture.png";
    case UniformType::SAMPLER2D:
        return "path/to/texture.png";
    case UniformType::SAMPLER3D:
        return "path/to/volume.raw";
    case UniformType::SAMPLERCUBE:
        return "path/to/cubemap.png";
    default:
        return "undefined";
    }
}

////////////////////////////////////////////////////////////////////////////////

bool is_valid_value(UniformType type, std::string& value)
{
    switch(type)
    {
    case UniformType::INT:
        return test_value_string<int>(value);
    case UniformType::FLOAT:
        return test_value_string<float>(value);
    case UniformType::BOOL:
        return test_value_string<bool>(value);
    case UniformType::VEC2:
        return test_value_string<math::vec2>(value);
    case UniformType::VEC3:
        return test_value_string<math::vec3>(value);
    case UniformType::VEC4:
        return test_value_string<math::vec4>(value);
    case UniformType::MAT3:
        if(test_value_string<math::mat3>(string_utils::replace(value, ";", "\n")))
        {
            string_utils::replace(value, "\n", ";");
            return true;
        }
        return false;
    case UniformType::MAT4:
        if(test_value_string<math::mat4>(string_utils::replace(value, ";", "\n")))
        {
            string_utils::replace(value, "\n", ";");
            return true;
        }
        return false;
    case UniformType::SAMPLER1D:
        return true;
    case UniformType::SAMPLER2D:
        return true;
    case UniformType::SAMPLER3D:
        return true;
    case UniformType::SAMPLERCUBE:
        return true;
    default:
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////

BufferComponentType get_type(BufferComponent component)
{
    switch(component)
    {
    case BufferComponent::I1:
        return TYPE_INTEGER;
    case BufferComponent::I2:
        return TYPE_INTEGER;
    case BufferComponent::I3:
        return TYPE_INTEGER;
    case BufferComponent::I4:
        return TYPE_INTEGER;
    case BufferComponent::U1:
        return TYPE_UNSIGNED;
    case BufferComponent::U2:
        return TYPE_UNSIGNED;
    case BufferComponent::U3:
        return TYPE_UNSIGNED;
    case BufferComponent::U4:
        return TYPE_UNSIGNED;
    case BufferComponent::H1:
        return TYPE_HALF;
    case BufferComponent::H2:
        return TYPE_HALF;
    case BufferComponent::H3:
        return TYPE_HALF;
    case BufferComponent::H4:
        return TYPE_HALF;
    case BufferComponent::F1:
        return TYPE_FLOAT;
    case BufferComponent::F2:
        return TYPE_FLOAT;
    case BufferComponent::F3:
        return TYPE_FLOAT;
    case BufferComponent::F4:
        return TYPE_FLOAT;
    case BufferComponent::DEPTH_16:
        return TYPE_DEPTH;
    case BufferComponent::DEPTH_24:
        return TYPE_DEPTH;
    default:
        return TYPE_NONE;
    }
}

////////////////////////////////////////////////////////////////////////////////

BufferComponent get_component(BufferComponentType type, unsigned components)
{
    if(type == TYPE_DEPTH)
        return BufferComponent::DEPTH_24;
    if(type == TYPE_NONE)
        return BufferComponent::NONE;

    return BufferComponent(type * 4 + components - 1);
}

////////////////////////////////////////////////////////////////////////////////

unsigned get_number_of_components(BufferComponent component)
{
    switch(component)
    {
    case BufferComponent::I1:
        return 1;
    case BufferComponent::I2:
        return 2;
    case BufferComponent::I3:
        return 3;
    case BufferComponent::I4:
        return 4;
    case BufferComponent::U1:
        return 1;
    case BufferComponent::U2:
        return 2;
    case BufferComponent::U3:
        return 3;
    case BufferComponent::U4:
        return 4;
    case BufferComponent::H1:
        return 1;
    case BufferComponent::H2:
        return 2;
    case BufferComponent::H3:
        return 3;
    case BufferComponent::H4:
        return 4;
    case BufferComponent::F1:
        return 1;
    case BufferComponent::F2:
        return 2;
    case BufferComponent::F3:
        return 3;
    case BufferComponent::F4:
        return 4;
    case BufferComponent::DEPTH_16:
        return 1;
    case BufferComponent::DEPTH_24:
        return 1;
    default:
        return 0;
    }
}

////////////////////////////////////////////////////////////////////////////////

std::set<std::string> list_output_types()
{
    std::set<std::string> result;

    for(int t(0); t < static_cast<int>(BufferComponent::NONE); ++t)
    {
        if(t != static_cast<int>(BufferComponent::DEPTH_16) && t != static_cast<int>(BufferComponent::DEPTH_24))
            result.insert(output_type_to_string(static_cast<BufferComponent>(t)));
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

std::set<std::string> list_uniform_types()
{
    std::set<std::string> result;

    for(int t(0); t < static_cast<int>(UniformType::NONE); ++t)
    {
        if(t != static_cast<int>(UniformType::SAMPLERCUBE))
            result.insert(uniform_type_to_string(static_cast<UniformType>(t)));
    }

    return result;
}

} // namespace enums

} // namespace gua
