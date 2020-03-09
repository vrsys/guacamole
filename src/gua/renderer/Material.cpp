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

#include <gua/renderer/Material.hpp>

#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/renderer/ShaderProgram.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

Material::Material(std::string const& shader_name)
    : shader_name_(shader_name), shader_cache_(nullptr), show_back_faces_(false), render_wireframe_(false), enable_early_fragment_test_(false)
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
      ,
      enable_virtual_texturing_(false)
#endif
{
    set_shader_name(shader_name_);
}

////////////////////////////////////////////////////////////////////////////////

Material::Material(Material const& copy)
    : shader_name_(copy.shader_name_), shader_cache_(copy.shader_cache_), uniforms_(copy.uniforms_), show_back_faces_(copy.show_back_faces_), render_wireframe_(copy.render_wireframe_)
{
}

////////////////////////////////////////////////////////////////////////////////

std::string const& Material::get_shader_name() const { return shader_name_; }

////////////////////////////////////////////////////////////////////////////////

void Material::set_shader_name(std::string const& name)
{
    std::lock_guard<std::mutex> lock(mutex_);
    shader_name_ = name;
    shader_cache_ = nullptr;

    auto shader(MaterialShaderDatabase::instance()->lookup(shader_name_));

    if(shader)
    {
        auto new_uniforms(shader->get_default_uniforms());

        for(auto const& old_uniform : uniforms_)
        {
            auto it(new_uniforms.find(old_uniform.first));
            if(it != new_uniforms.end())
            {
                it->second = old_uniform.second;
            }
        }

        uniforms_ = new_uniforms;
    }
}

////////////////////////////////////////////////////////////////////////////////

void Material::rename_existing_shader(std::string const& name)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto shader_with_old_name(MaterialShaderDatabase::instance()->lookup(shader_name_));

    shader_name_ = name;
    shader_cache_ = nullptr;

    auto shader_with_new_name = std::make_shared<MaterialShader>(shader_name_, shader_with_old_name->get_description());
    MaterialShaderDatabase::instance()->add(shader_with_new_name);
}

////////////////////////////////////////////////////////////////////////////////

MaterialShader* Material::get_shader() const
{
    // boost::unique_lock<boost::shared_mutex> lock(mutex_);
    if(!shader_cache_)
    {
        shader_cache_ = MaterialShaderDatabase::instance()->lookup(shader_name_).get();
    }

    return shader_cache_;
}

////////////////////////////////////////////////////////////////////////////////

std::map<std::string, ViewDependentUniform> const& Material::get_uniforms() const { return uniforms_; }

////////////////////////////////////////////////////////////////////////////////

void Material::apply_uniforms(RenderContext const& ctx, ShaderProgram* shader, int view) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    for(auto const& uniform : uniforms_)
    {
        uniform.second.apply(ctx, uniform.first, view, shader->get_program());
    }
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& Material::serialize_uniforms_to_stream(std::ostream& os) const
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

void Material::set_uniforms_from_serialized_string(std::string const& value)
{
    auto tokens(string_utils::split(value, ';'));

    for(auto& token : tokens)
    {
        auto parts(string_utils::split(token, '#'));
        set_uniform(parts[0], ViewDependentUniform::create_from_serialized_string(parts[1]));
    }
}

////////////////////////////////////////////////////////////////////////////////

std::ostream& operator<<(std::ostream& os, Material const& val) { return val.serialize_uniforms_to_stream(os); }

////////////////////////////////////////////////////////////////////////////////

template <>
Material& Material::set_uniform<std::string>(std::string const& name, std::string const& tex_name, int view_id)
{
    auto uniform(uniforms_.find(name));
    if(!TextureDatabase::instance()->contains(tex_name))
        TextureDatabase::instance()->load(tex_name);

    if(uniform != uniforms_.end())
    {
        uniform->second.set(view_id, tex_name);
    }
    else
    {
        ViewDependentUniform tmp;
        tmp.set(UniformValue(tex_name));
        tmp.set(view_id, UniformValue(tex_name));
        uniforms_[name] = tmp;
    }
    return *this;
}

template <>
Material& Material::set_uniform<std::string>(std::string const& name, std::string const& tex_name)
{
    if(!TextureDatabase::instance()->contains(tex_name))
        TextureDatabase::instance()->load(tex_name);
    return set_uniform(name, ViewDependentUniform(UniformValue(uniform_compatible_type(tex_name))));
}

} // namespace gua
