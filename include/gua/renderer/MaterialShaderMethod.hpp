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

#ifndef GUA_MATERIAL_SHADER_METHOD_HPP
#define GUA_MATERIAL_SHADER_METHOD_HPP

#include <gua/renderer/ViewDependentUniform.hpp>

#include <iostream>
#include <string>
#include <memory>
#include <vector>

namespace gua
{
class GUA_DLL MaterialShaderMethod
{
  public:
    MaterialShaderMethod(std::string const& name = "");
    ~MaterialShaderMethod() = default;

    MaterialShaderMethod& load_from_file(std::string const& file_name);

    MaterialShaderMethod& load_from_json(std::string const& json_string);

    MaterialShaderMethod& set_name(std::string const& name);
    std::string const& get_name() const;

    MaterialShaderMethod& set_source(std::string const& source);
    std::string const& get_source() const;

    template <typename T>
    MaterialShaderMethod& set_uniform(std::string const& name, T const& value)
    {
        return set_uniform(name, ViewDependentUniform(UniformValue(uniform_compatible_type(value))));
    }

    MaterialShaderMethod& set_uniform(std::string const& name, ViewDependentUniform const& uniform)
    {
        uniforms_[name] = uniform;
        return *this;
    }

    std::map<std::string, ViewDependentUniform> const& get_uniforms() const;

    std::ostream& serialize_uniforms_to_stream(std::ostream& os) const;
    void set_uniforms_from_serialized_string(std::string const& value);

  private:
    std::string name_;
    std::string source_;
    std::map<std::string, ViewDependentUniform> uniforms_;
};

} // namespace gua

#endif // GUA_MATERIAL_SHADER_METHOD_HPP
