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

#ifndef GUA_G_BUFFER_NURBS_UBER_SHADER_HPP
#define GUA_G_BUFFER_NURBS_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/UberShader.hpp>

namespace gua {

/**
 *
 */
class GBufferNURBSUberShader : public UberShader {
 public:

   enum pass {
     transform_feedback_pass = 0,
     final_pass = 1
   };

  /**
   * Default constructor.
   *
   * Creates a new GBufferNURBSUberShader multi-pass routine.
   */
  GBufferNURBSUberShader();

  /**
   * Destructor
   *
   * Cleans all associated memory.
   */
  virtual ~GBufferNURBSUberShader();

  /**
   *
   */
  void create(std::set<std::string> const& material_names);

 private:  // auxiliary methods

  std::string const _transform_feedback_vertex_shader() const;
  std::string const _transform_feedback_geometry_shader() const;
  std::string const _transform_feedback_tess_control_shader() const;
  std::string const _transform_feedback_tess_evaluation_shader() const;

  std::string const _final_vertex_shader() const;
  std::string const _final_tess_control_shader() const;
  std::string const _final_tess_evaluation_shader() const;
  std::string const _final_geometry_shader() const;
  std::string const _final_fragment_shader() const;


 private:  // attributes

  UberShaderFactory* vertex_shader_factory_;
  UberShaderFactory* fragment_shader_factory_;
};

}

#endif  // GUA_G_BUFFER_NURBS_UBER_SHADER_HPP
