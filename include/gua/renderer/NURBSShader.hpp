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

#ifndef GUA_NURBS_SHADER_HPP
#define GUA_NURBS_SHADER_HPP

#include <string>

// guacamole headers
#include <gua/renderer/ShaderProgram.hpp>

namespace gua {

/**
 * non-instantiable interface that provides GLSL functionality for NURBS
 * rendering
 */
class NURBSShader {
 private:

  /**
   *
   */
  NURBSShader();

 public:
  /**
   * Destructor
   */
  virtual ~NURBSShader();

 public:

  static std::string const surface_horner_evaluation();
  static std::string const horner_simple();
  static std::string const horner_derivatives();
  static std::string const curve_horner_evaluation();
  static std::string const trim_classification();

  static std::string const control_polygon_length();
  static std::string const edge_length();
  static std::string const edge_tess_level();
  static std::string const inner_tess_level();
  static std::string const to_screen_space();
  static std::string const frustum_cull();
  static std::string const is_inside();

  // fragment-based trimming based on binary clustered contour
  static std::string const binary_search();
  static std::string const trimming_helper_methods();
  static std::string const bisect_contour();
  static std::string const bisect_curve();
  static std::string const contour_binary_search();
  static std::string const contour_based_trimming();

 private:  // attributes
};

}

#endif  // GUA_NURBS_SHADER_HPP
