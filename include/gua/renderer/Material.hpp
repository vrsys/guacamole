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

#ifndef GUA_MATERIAL_HPP
#define GUA_MATERIAL_HPP

#include <gua/renderer/ViewDependentUniform.hpp>

#include <mutex>
#include <string>
#include <vector>

namespace gua
{
class MaterialShader;
class ShaderProgram;

class GUA_DLL Material
{
  public:
    Material(std::string const& shader_name = "gua_default_material");
    Material(Material const& copy);

    std::string const& get_shader_name() const;
    void set_shader_name(std::string const&);

    void rename_existing_shader(std::string const& name);

    MaterialShader* get_shader() const;

    template <typename T>
    Material& set_uniform(std::string const& name, T const& value)
    {
        return set_uniform(name, ViewDependentUniform(UniformValue(uniform_compatible_type(value))));
    }

    template <typename T>
    Material& set_uniform(std::string const& name, T const& value, int view_id)
    {
        auto val = uniform_compatible_type(value);
        auto uniform(uniforms_.find(name));

        if(uniform != uniforms_.end())
        {
            uniform->second.set(view_id, val);
        }
        else
        {
            ViewDependentUniform tmp;
            tmp.set(UniformValue(val));
            tmp.set(view_id, UniformValue(val));
            uniforms_[name] = tmp;
        }
        return *this;
    }

    Material& reset_uniform(std::string const& name, int view_id)
    {
        uniforms_[name].reset(view_id);
        return *this;
    }

    std::map<std::string, ViewDependentUniform> const& get_uniforms() const;

    Material& set_show_back_faces(bool value)
    {
        show_back_faces_ = value;
        return *this;
    }

    bool get_show_back_faces() const { return show_back_faces_; }

    Material& set_render_wireframe(bool value)
    {
        render_wireframe_ = value;
        return *this;
    }

    bool get_render_wireframe() const { return render_wireframe_; }

    Material& set_enable_early_fragment_test(bool value) {
        enable_early_fragment_test_ = value;
        return *this;
    }

    bool get_enable_early_fragment_test() const {
        return enable_early_fragment_test_;
    }

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    Material& set_enable_virtual_texturing(bool value)
    {
        enable_virtual_texturing_ = value;
        return *this;
    }

    bool get_enable_virtual_texturing() const { return enable_virtual_texturing_; }
#endif

    void apply_uniforms(RenderContext const& ctx, ShaderProgram* shader, int view) const;

    std::ostream& serialize_uniforms_to_stream(std::ostream& os) const;
    void set_uniforms_from_serialized_string(std::string const& value);

    std::string get_sampler(std::string const& name, int view_id)
    {
        auto iter = uniforms_.find(name);
        if(iter == uniforms_.end())
        {
            return std::string("");
        }
        else
        {
            return boost::get<std::string>(iter->second.get(view_id).data);
        }
    }

    std::string get_sampler(std::string const& name)
    {
        auto iter = uniforms_.find(name);
        if(iter == uniforms_.end())
        {
            return std::string("");
        }
        else
        {
            return boost::get<std::string>(iter->second.get().data);
        }
    }

    math::mat4f get_mat4(std::string const& name, int view_id)
    {
        auto iter = uniforms_.find(name);
        if(iter == uniforms_.end())
        {
            return math::mat4f{};
        }
        else
        {
            return boost::get<math::mat4f>(iter->second.get(view_id).data);
        }
    }

    math::mat4f get_mat4(std::string const& name)
    {
        auto iter = uniforms_.find(name);
        if(iter == uniforms_.end())
        {
            return math::mat4f{};
        }
        else
        {
            return boost::get<math::mat4f>(iter->second.get().data);
        }
    }

  private:
    Material& set_uniform(std::string const& name, ViewDependentUniform const& value)
    {
        uniforms_[name] = value;
        return *this;
    }

    friend class MaterialShader;

    std::string shader_name_;
    mutable MaterialShader* shader_cache_;
    std::map<std::string, ViewDependentUniform> uniforms_;
    bool show_back_faces_;
    bool render_wireframe_;
    bool enable_early_fragment_test_;

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    bool enable_virtual_texturing_;
#endif

    mutable std::mutex mutex_;
};

template <>
Material& Material::set_uniform<std::string>(std::string const& name, std::string const& val, int view_id);

template <>
Material& Material::set_uniform<std::string>(std::string const& name, std::string const& value);

template GUA_DLL Material& Material::set_uniform<int>(std::string const& name, int const& value);
template GUA_DLL Material& Material::set_uniform<float>(std::string const& name, float const& value);
template GUA_DLL Material& Material::set_uniform<math::vec2>(std::string const& name, math::vec2 const& value);
template GUA_DLL Material& Material::set_uniform<math::vec3>(std::string const& name, math::vec3 const& value);
template GUA_DLL Material& Material::set_uniform<math::vec4>(std::string const& name, math::vec4 const& value);
template GUA_DLL Material& Material::set_uniform<std::string>(std::string const& name, std::string const& value);
template GUA_DLL Material& Material::set_uniform<std::string>(std::string const& name, std::string const& value, int view_id);

// operators
std::ostream& operator<<(std::ostream& os, Material const& val);

} // namespace gua

#endif // GUA_MATERIAL_HPP
