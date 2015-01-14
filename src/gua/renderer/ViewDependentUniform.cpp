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

#include <gua/renderer/ViewDependentUniform.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
ViewDependentUniform::ViewDependentUniform(UniformValue const& value):
  default_(value)
  {}

////////////////////////////////////////////////////////////////////////////////
UniformValue const& ViewDependentUniform::get() const{
  return default_;
}

////////////////////////////////////////////////////////////////////////////////
UniformValue const& ViewDependentUniform::get(int view) const{
  auto overwrite(uniforms_.find(view));
  if (overwrite != uniforms_.end()) {
    return overwrite->second;
  }
  return default_;
}

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::set(UniformValue const& value) {
  default_ = value;
}

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::set(int view, UniformValue const& value) {
  uniforms_[view] = value;
}

////////////////////////////////////////////////////////////////////////////////
void ViewDependentUniform::apply(RenderContext const& ctx,
                                 std::string const& name, int view,
                                 scm::gl::program_ptr const& prog,
                                 unsigned location) const {
  auto overwrite(uniforms_.find(view));
  if (overwrite != uniforms_.end()) {
    overwrite->second.apply(ctx, name, prog, location);
  } else {
    default_.apply(ctx, name, prog, location);
  }
}

}
