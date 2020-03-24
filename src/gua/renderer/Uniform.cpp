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
#include <gua/renderer/Uniform.hpp>

#include <gua/math/math.hpp>

namespace gua
{
struct GUA_DLL ApplyUniform : public boost::static_visitor<>
{
    ApplyUniform(RenderContext const& context, std::string const& str, scm::gl::program_ptr const& p, unsigned l = 0) : ctx(context), name(str), prog(p), location(l) {}

    RenderContext const& ctx;
    std::string const& name;
    scm::gl::program_ptr const& prog;
    unsigned location;

    void operator()(int value) const { prog->uniform(name, location, value); }
    void operator()(bool value) const { prog->uniform(name, location, value); }
    void operator()(float value) const { prog->uniform(name, location, value); }
    void operator()(math::mat3f const& value) const { prog->uniform(name, location, value); }
    void operator()(math::mat4f const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec2f const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec3f const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec4f const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec2i const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec3i const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec4i const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec2ui const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec3ui const& value) const { prog->uniform(name, location, value); }
    void operator()(math::vec4ui const& value) const { prog->uniform(name, location, value); }
    void operator()(std::string const& tex_name) const
    {
        if(tex_name == "0")
        {
            prog->uniform(name, location, math::vec2ui(0, 0));
        }
        else
        {
            auto texture = TextureDatabase::instance()->lookup(tex_name);
            if(!texture)
            {
                TextureDatabase::instance()->load(tex_name);
                texture = TextureDatabase::instance()->lookup(tex_name);
            }
            if(texture)
            {
                int32_t global_tex_id = TextureDatabase::instance()->get_global_texture_id_by_path(tex_name);
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
                prog->uniform("gua_current_vt_idx", location, global_tex_id);
#endif
                prog->uniform(name, location, texture->get_handle(ctx));
            }
        }
    }
};

void UniformValue::apply(RenderContext const& ctx, std::string const& name, scm::gl::program_ptr const& prog, unsigned location) const
{
    if(prog != nullptr)
        boost::apply_visitor(ApplyUniform(ctx, name, prog, location), data);
}

template <>
void UniformValue::write_bytes_impl<bool>(UniformValue const* self, RenderContext const& ctx, char* target)
{
    memcpy(target, &boost::get<bool>(self->data), sizeof(int));
}

template <>
void UniformValue::write_bytes_impl<std::string>(UniformValue const* self, RenderContext const& ctx, char* target)
{
    auto tex_name(boost::get<std::string>(self->data));
    if(tex_name == "0")
    {
        math::vec2ui handle(0, 0);
        memcpy(target, &handle, sizeof(math::vec2ui));
    }
    else
    {
        auto texture(TextureDatabase::instance()->lookup(tex_name));
        if(!texture)
        {
            TextureDatabase::instance()->load(tex_name);
            texture = TextureDatabase::instance()->lookup(tex_name);
        }
        if(texture)
        {
            auto& handle(texture->get_handle(ctx));
            memcpy(target, &handle, sizeof(math::vec2ui));
        }
    }
}

UniformValue UniformValue::create_from_string_and_type(std::string const& value, UniformType const& ty)
{
    switch(ty)
    {
    case UniformType::INT:
        return UniformValue(string_utils::from_string<int>(value));
    case UniformType::FLOAT:
        return UniformValue(string_utils::from_string<float>(value));
    case UniformType::BOOL:
        return UniformValue(string_utils::from_string<bool>(value));
    case UniformType::VEC2:
        return UniformValue(string_utils::from_string<scm::math::vec2f>(value));
    case UniformType::VEC3:
        return UniformValue(string_utils::from_string<scm::math::vec3f>(value));
    case UniformType::VEC4:
        return UniformValue(string_utils::from_string<scm::math::vec4f>(value));
    case UniformType::VEC2I:
        return UniformValue(string_utils::from_string<scm::math::vec2i>(value));
    case UniformType::VEC3I:
        return UniformValue(string_utils::from_string<scm::math::vec3i>(value));
    case UniformType::VEC4I:
        return UniformValue(string_utils::from_string<scm::math::vec4i>(value));
    case UniformType::VEC2UI:
        return UniformValue(string_utils::from_string<scm::math::vec2ui>(value));
    case UniformType::VEC3UI:
        return UniformValue(string_utils::from_string<scm::math::vec3ui>(value));
    case UniformType::VEC4UI:
        return UniformValue(string_utils::from_string<scm::math::vec4ui>(value));
    case UniformType::MAT3:
        return UniformValue(string_utils::from_string<scm::math::mat3f>(value));
    case UniformType::MAT4:
        return UniformValue(string_utils::from_string<scm::math::mat4f>(value));
    // case UniformType::SAMPLER1D:
    //  return UniformValue(value);
    case UniformType::SAMPLER2D:
        return UniformValue(value);
        // case UniformType::SAMPLER3D:
        //  return UniformValue(value);
        // case UniformType::SAMPLERCUBE:
        //  return UniformValue(value);
        // case NONE
    case UniformType::SAMPLER1D:
    case UniformType::SAMPLER3D:
    case UniformType::SAMPLERCUBE:
    case UniformType::NONE:
        throw std::runtime_error("UniformValue::create_from_string_and_type(): Invalid type");
    }
    throw std::runtime_error("UniformValue::create_from_string_and_type(): Invalid type");
}

UniformValue UniformValue::create_from_strings(std::string const& value, std::string const& ty)
{
    if(auto t = gua::enums::parse_uniform_type(ty))
    {
        return create_from_string_and_type(value, *t);
    }
    else
    {
        throw std::runtime_error("UniformValue::create_from_strings(): Invalid type");
    }
}

UniformValue UniformValue::create_from_serialized_string(std::string const& value)
{
    auto tokens(string_utils::split(value, '|'));
    return create_from_strings(tokens[1], tokens[0]);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<int>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::INT) << "|" << boost::get<int>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<bool>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::BOOL) << "|" << boost::get<bool>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<float>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::FLOAT) << "|" << boost::get<float>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::mat3f>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::MAT3) << "|" << boost::get<math::mat3f>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::mat4f>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::MAT4) << "|" << boost::get<math::mat4f>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec2f>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC2) << "|" << boost::get<math::vec2f>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec3f>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC3) << "|" << boost::get<math::vec3f>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec4f>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC4) << "|" << boost::get<math::vec4f>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec2i>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC2I) << "|" << boost::get<math::vec2i>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec3i>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC3I) << "|" << boost::get<math::vec3i>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec4i>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC4I) << "|" << boost::get<math::vec4i>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec2ui>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC2UI) << "|" << boost::get<math::vec2ui>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec3ui>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC3UI) << "|" << boost::get<math::vec3ui>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<math::vec4ui>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::VEC4UI) << "|" << boost::get<math::vec4ui>(self->data);
}

template <>
std::ostream& UniformValue::serialize_to_stream_impl<std::string>(UniformValue const* self, std::ostream& os)
{
    return os << enums::uniform_type_to_string(UniformType::SAMPLER2D) << "|" << boost::get<std::string>(self->data);
}

std::ostream& operator<<(std::ostream& os, UniformValue const& val) { return val.serialize_to_stream(os); }

} // namespace gua
