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

#ifndef GUA_PBR_UBER_SHADER_HPP
#define GUA_PBR_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/GeometryUberShader.hpp>

namespace gua {

class PBRUberShader : public GeometryUberShader {

 public:

  void              create  (std::set<std::string> const& material_names);

  bool              upload_to (RenderContext const& context) const;

  /*virtual*/ stage_mask const get_stage_mask() const;

  /*virtual*/ void  preframe  (RenderContext const& context) const;

  /*virtual*/ void  predraw   (RenderContext const& ctx,
                               std::string const& filename,
                               std::string const& material_name,
                               scm::math::mat4 const& model_matrix,
                               scm::math::mat4 const& normal_matrix,
                               Frustum const& /*frustum*/) const;

  /*virtual*/ void  draw      (RenderContext const& ctx,
                              std::string const& filename,
                              std::string const& material_name,
                              scm::math::mat4 const& model_matrix,
                              scm::math::mat4 const& normal_matrix,
                              Frustum const& /*frustum*/) const;

  /*virtual*/ void  postdraw (RenderContext const& ctx,
                              std::string const& filename,
                              std::string const& material_name,
                              scm::math::mat4 const& model_matrix,
                              scm::math::mat4 const& normal_matrix,
                              Frustum const& /*frustum*/) const;

  /*virtual*/ void  postframe (RenderContext const& context) const;

 private: //auxialiary methods
  std::string const forward_point_rendering_vertex_shader() const;
  std::string const forward_point_rendering_fragment_shader() const;

 private:  //member variables

  mutable std::vector<scm::gl::rasterizer_state_ptr> change_point_size_in_shader_state_;
};

}

#endif  // GUA_PBR_UBER_SHADER_HPP
