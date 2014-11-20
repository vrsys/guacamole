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
#ifndef GUA_NURBSPASS_HPP
#define GUA_NURBSPASS_HPP

// guacamole headers
#include <gua/renderer/PipelinePass.hpp>

namespace gua {

  class NURBSPassDescription : public PipelinePassDescription {

  public : // typedefs, enums

   enum pass {
     tesselation_pre_pass = 0,
     tesselation_final_pass = 1,
     raycasting = 2
   };

  public :

  void init_process_callback();

  void draw     () const;

 private:  // auxiliary methods

  std::string _transform_feedback_vertex_shader() const;
  std::string _transform_feedback_geometry_shader() const;
  std::string _transform_feedback_tess_control_shader() const;
  std::string _transform_feedback_tess_evaluation_shader() const;

  std::string _final_vertex_shader() const;
  std::string _final_tess_control_shader() const;
  std::string _final_tess_evaluation_shader() const;
  std::string _final_geometry_shader() const;
  std::string _final_fragment_shader() const;

  std::string _raycast_vertex_shader() const;
  std::string _raycast_fragment_shader() const;

  void        _insert_generic_per_vertex_code(std::string& code) const;
  void        _insert_generic_per_fragment_code(std::string& rcode) const;

 private:  // attributes

};

}

#endif  // GUA_NURBSPASS_HPP
