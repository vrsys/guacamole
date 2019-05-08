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

#include <gua/renderer/MaterialShader.hpp>

#include <gua/renderer/ShaderProgram.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/string_utils.hpp>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////
MaterialShader::MaterialShader(std::string const& name, std::shared_ptr<MaterialShaderDescription> const& desc) : desc_(desc), name_(name)
{
    auto v_methods = desc_->get_vertex_methods();
    auto f_methods = desc_->get_fragment_methods();

    for(auto const& method : v_methods)
    {
        for(auto const& uniform : method->get_uniforms())
        {
            default_uniforms_[uniform.first] = ViewDependentUniform(uniform.second);
        }
    }

    for(auto const& method : f_methods)
    {
        for(auto const& uniform : method->get_uniforms())
        {
            default_uniforms_[uniform.first] = ViewDependentUniform(uniform.second);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<MaterialShaderDescription> const& MaterialShader::get_description() const { return desc_; }

////////////////////////////////////////////////////////////////////////////////
std::string const& MaterialShader::get_name() const { return name_; }

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> MaterialShader::make_new_material() const { return std::make_shared<Material>(name_); }

////////////////////////////////////////////////////////////////////////////////
std::map<std::string, ViewDependentUniform> const& MaterialShader::get_default_uniforms() const { return default_uniforms_; }

////////////////////////////////////////////////////////////////////////////////
std::list<std::shared_ptr<MaterialShaderMethod>> const& MaterialShader::get_vertex_methods() const { return desc_->get_vertex_methods(); }

////////////////////////////////////////////////////////////////////////////////
std::list<std::shared_ptr<MaterialShaderMethod>> const& MaterialShader::get_fragment_methods() const { return desc_->get_fragment_methods(); }

SubstitutionMap MaterialShader::generate_substitution_map() const
{
    SubstitutionMap smap;
    std::stringstream sstr;

    const auto& v_methods = get_vertex_methods();
    const auto& f_methods = get_fragment_methods();

    // uniform substitutions
    for(auto const& uniform : default_uniforms_)
    {
        sstr << "uniform " << uniform.second.get().get_glsl_type() << " " << uniform.first << ";" << std::endl;
    }
    sstr << std::endl;
    smap["material_uniforms"] = sstr.str();
    smap["material_input"] = "";
    sstr.str("");

    // material methods substitutions
    for(auto const& method : v_methods)
    {
        sstr << method->get_source() << std::endl;
    }
    smap["material_method_declarations_vert"] = sstr.str();
    sstr.str("");

    for(auto& method : f_methods)
    {
        sstr << method->get_source() << std::endl;
    }
    smap["material_method_declarations_frag"] = sstr.str();
    sstr.str("");

    // material method calls substitutions
    for(auto const& method : v_methods)
    {
        sstr << method->get_name() << "();" << std::endl;
    }
    smap["material_method_calls_vert"] = sstr.str();
    sstr.str("");

    for(auto& method : f_methods)
    {
        sstr << method->get_name() << "();" << std::endl;
    }
    smap["material_method_calls_frag"] = sstr.str();

    smap["enable_virtual_texturing"] = "0";
    return smap;
}

} // namespace gua
