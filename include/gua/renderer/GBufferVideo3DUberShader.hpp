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

#ifndef GUA_G_BUFFER_VIDEO3D_UBER_SHADER_HPP
#define GUA_G_BUFFER_VIDEO3D_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/UberShader.hpp>
 #include <gua/renderer/UberShaderFactory.hpp>

namespace gua {

class GBufferVideo3DUberShader : public UberShader {
 public:

  void create(std::set<std::string> const& material_names);

 private:
  std::string const _final_vertex_shader(UberShaderFactory const& vshader_factory,
  	                                       LayerMapping const& vshader_output_mapping) const;
  std::string const _final_geometry_shader(UberShaderFactory const& vshader_factory,
                                           LayerMapping const& vshader_output_mapping) const;
  std::string const _final_fragment_shader(UberShaderFactory const& fshader_factory,
  	                                       LayerMapping const& vshader_output_mapping) const;

};

}

#endif  // GUA_G_BUFFER_VIDEO3D_UBER_SHADER_HPP
