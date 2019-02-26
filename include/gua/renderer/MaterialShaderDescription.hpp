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

#ifndef GUA_MATERIAL_SHADER_DESCRIPTION_HPP
#define GUA_MATERIAL_SHADER_DESCRIPTION_HPP

#include <gua/renderer/MaterialShaderMethod.hpp>

#include <list>

namespace gua
{
class GUA_DLL MaterialShaderDescription
{
  public:
    MaterialShaderDescription() = default;
    MaterialShaderDescription(std::string const& file);

    void load_from_file(std::string const& file_name);
    void load_from_json(std::string const& json);

    MaterialShaderDescription& add_vertex_method(std::shared_ptr<MaterialShaderMethod> const& method);
    MaterialShaderDescription& add_fragment_method(std::shared_ptr<MaterialShaderMethod> const& method);

    std::list<std::shared_ptr<MaterialShaderMethod>> const& get_vertex_methods() const;
    std::list<std::shared_ptr<MaterialShaderMethod>> const& get_fragment_methods() const;

    MaterialShaderDescription& clear_vertex_methods();
    MaterialShaderDescription& clear_fragment_methods();

  private:
    std::list<std::shared_ptr<MaterialShaderMethod>> vertex_methods_;
    std::list<std::shared_ptr<MaterialShaderMethod>> fragment_methods_;
};

} // namespace gua

#endif // GUA_MATERIAL_SHADER_DESCRIPTION_HPP
