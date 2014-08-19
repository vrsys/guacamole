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

#ifndef GUA_NURBS_RESSOURCE_HPP
#define GUA_NURBS_RESSOURCE_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/GeometryRessource.hpp>
#include <gua/renderer/nurbs_geometry/NURBSData.hpp>

// external headers
#include <string>
#include <vector>
#include <mutex>

#include <scm/core/math.h>

#include <scm/gl_core/gl_core_fwd.h>
#include <scm/gl_core/data_types.h>
#include <scm/gl_core/state_objects.h>

#include <scm/gl_util/primitives/primitives_fwd.h>
#include <scm/gl_util/primitives/geometry.h>

#include <scm/core/platform/platform.h>
#include <scm/core/utilities/platform_warning_disable.h>

namespace gua {

class GUA_NURBS_DLL NURBSRessource : public GeometryRessource {

 public:
   NURBSRessource(std::shared_ptr<TrimmedBezierSurfaceObject> const& object,
                  scm::gl::fill_mode in_fill_mode = scm::gl::FILL_SOLID,
                  std::size_t max_transform_feedback_buffer_size = 200000000);  // 200MB TXFB

   ~NURBSRessource();

  /*virtual*/ void draw(RenderContext const& context) const;

  /*virtual*/ void predraw(RenderContext const& context) const;

  /*virtual*/ void update_bounding_box() const;

  /*virtual*/ std::shared_ptr<GeometryUberShader> create_ubershader() const;

  void ray_test(Ray const& ray, PickResult::Options options,
                node::Node* owner, std::set<PickResult>& hits) {}

  scm::gl::buffer_ptr const& vertex_buffer() const;
  scm::gl::buffer_ptr const& index_buffer() const;
  scm::gl::vertex_array_ptr const& vertex_array() const;

 private:

  NURBSData* _data;
  scm::gl::fill_mode _fill_mode;

  //Texture2D Buffer for Parametric Data
  mutable std::vector<scm::gl::texture_buffer_ptr> _parametric_texture_buffer;

  //Texture2D Buffer for Attributes
  mutable std::vector<scm::gl::texture_buffer_ptr> _attribute_texture_buffer;

  //Texture2D Buffer for Domain
  mutable std::vector<scm::gl::texture_buffer_ptr> _domain_texture_buffer;

  //Texture2D Buffers for Trim Data
  mutable std::vector<scm::gl::texture_buffer_ptr>
      _trim_partition_texture_buffer;
  mutable std::vector<scm::gl::texture_buffer_ptr>
      _trim_contourlist_texture_buffer;
  mutable std::vector<scm::gl::texture_buffer_ptr>
      _trim_curvelist_texture_buffer;
  mutable std::vector<scm::gl::texture_buffer_ptr>
      _trim_curvedata_texture_buffer;
  mutable std::vector<scm::gl::texture_buffer_ptr>
      _trim_pointdata_texture_buffer;

  //Vertex Array and Buffer
  mutable std::vector<scm::gl::vertex_array_ptr> _vertex_array;
  mutable std::vector<scm::gl::buffer_ptr> _vertex_buffer;
  mutable std::vector<scm::gl::buffer_ptr> _index_buffer;

  mutable std::vector<scm::gl::sampler_state_ptr> _sstate;
  mutable std::vector<scm::gl::rasterizer_state_ptr> _rstate_ms_solid;
  mutable std::vector<scm::gl::rasterizer_state_ptr> _rstate_ms_wireframe;
  mutable std::vector<scm::gl::rasterizer_state_ptr> _rstate_ms_point;
  mutable std::vector<scm::gl::blend_state_ptr> _bstate_no_blend;

  /////////////////////////////////////////////////////////////////////////////////////////////
  // Transformation Feedback Specific Members
  /////////////////////////////////////////////////////////////////////////////////////////////

  mutable std::vector<scm::gl::transform_feedback_ptr> _transform_feedback;
  mutable std::vector<scm::gl::vertex_array_ptr> _transform_feedback_vao;
  mutable std::vector<scm::gl::buffer_ptr> _transform_feedback_vbo;
  /////////////////////////////////////////////////////////////////////////////////////////////

 private:  // helper methods

  void upload_to(RenderContext const& context) const;
  void initialize_texture_buffers(RenderContext const& context) const;
  void initialize_transform_feedback(RenderContext const& context) const;
  void initialize_vertex_data(RenderContext const& context) const;

 private:  // attributes

  std::size_t _max_transform_feedback_buffer_size;
  mutable std::mutex upload_mutex_;
};

}  //namespace gua

#endif // GUA_NURBS_RESSOURCE_HPP
