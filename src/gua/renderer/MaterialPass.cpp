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
#include <gua/renderer/MaterialPass.hpp>

// guacamole headers

namespace gua {

////////////////////////////////////////////////////////////////////////////////

MaterialPass::MaterialPass(std::string const& name) :
  name_(name) {}

////////////////////////////////////////////////////////////////////////////////

MaterialPass& MaterialPass::set_source(std::string const& source) {
  source_ = source;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& MaterialPass::get_name() const {
  return name_;
}

////////////////////////////////////////////////////////////////////////////////

std::string const& MaterialPass::get_source() const {
  return source_;
}

////////////////////////////////////////////////////////////////////////////////

std::unordered_map<std::string, std::shared_ptr<UniformValueBase>> const&
MaterialPass::get_uniforms() const {
  return uniforms_;
}

}
