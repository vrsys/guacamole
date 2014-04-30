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

#ifndef GUA_TRI_MESH_UBER_SHADER_HPP
#define GUA_TRI_MESH_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/GeometryUberShader.hpp>

namespace gua {

class TriMeshUberShader : public GeometryUberShader {

 public:

  void              create  (std::set<std::string> const& material_names);

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
};

}

#endif  // GUA_TRI_MESH_UBER_SHADER_HPP
