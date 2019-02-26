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

#ifndef GUA_VIEW_DEPENDENT_UNIFORM_HPP
#define GUA_VIEW_DEPENDENT_UNIFORM_HPP

#include <gua/renderer/Uniform.hpp>

#include <string>
#include <vector>

namespace gua
{
class GUA_DLL ViewDependentUniform
{
  public:
    ViewDependentUniform(UniformValue const& value = UniformValue());

    UniformValue const& get() const;
    UniformValue const& get(int view) const;

    void set(UniformValue const& value);
    void set(int view, UniformValue const& value);

    void reset(int view);

    void apply(RenderContext const& ctx, std::string const& name, int view, scm::gl::program_ptr const& prog, unsigned location = 0) const;

    std::ostream& serialize_to_stream(std::ostream& os) const;

    static ViewDependentUniform create_from_serialized_string(std::string const& value);

  private:
    UniformValue default_;
    std::map<int, UniformValue> uniforms_;
};

// operators
std::ostream& operator<<(std::ostream& os, ViewDependentUniform const& val);

} // namespace gua

#endif // GUA_VIEW_DEPENDENT_UNIFORM_HPP
