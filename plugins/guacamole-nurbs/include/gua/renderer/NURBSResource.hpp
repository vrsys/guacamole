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

#ifndef GUA_NURBS_RESOURCE_HPP
#define GUA_NURBS_RESOURCE_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/renderer/detail/NURBSData.hpp>

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

  namespace node {
    class NURBSNode;
  };

  template <typename T>
  std::size_t size_in_bytes(T const& container)
  {
    using value_type = typename T::value_type;
    return container.size() * sizeof(value_type);
  };

class GUA_NURBS_DLL NURBSResource : public GeometryResource {

 public: // constants

   struct texture_buffer_binding {
     scm::gl::texture_buffer_ptr buffer;
     unsigned texunit;
   };
   struct ssbo_binding {
     scm::gl::buffer_ptr buffer;
     unsigned unit;
   };

   static std::size_t const MAX_XFB_BUFFER_SIZE_IN_BYTES = 200000000; // 200MB temporary XFB Buffer

 public : // c'tor / d'tor

   NURBSResource(std::shared_ptr<gpucast::beziersurfaceobject> const& object,
                  unsigned pre_subdivision_u,
                  unsigned pre_subdivision_v,
                  unsigned trim_resolution,
                  scm::gl::fill_mode in_fill_mode = scm::gl::FILL_SOLID
                  //scm::gl::fill_mode in_fill_mode = scm::gl::FILL_WIREFRAME
                  );

 public : // methods

  /*virtual*/ void predraw(RenderContext const& context, bool cull_face) const;

  /*virtual*/ void draw(RenderContext const& context, bool raycasting, bool cull_face) const;

  void ray_test(Ray const& ray, int options,
                node::Node* owner, std::set<PickResult>& hits) {}

  scm::gl::buffer_ptr const& vertex_buffer() const;
  scm::gl::buffer_ptr const& index_buffer() const;
  scm::gl::vertex_array_ptr const& vertex_array() const;

 private:
   
  /////////////////////////////////////////////////////////////////////////////////////////////
  // CPU ressources
  /////////////////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<NURBSData> _data;

  scm::gl::fill_mode         _fill_mode;

  /////////////////////////////////////////////////////////////////////////////////////////////
  // GPU ressources
  /////////////////////////////////////////////////////////////////////////////////////////////

  // array and texture buffers for adaptive tesselation
  struct surface_tesselation_buffer {
    scm::gl::vertex_array_ptr     vertex_array;
                                   
    scm::gl::buffer_ptr           vertex_buffer;
    scm::gl::buffer_ptr           index_buffer;
    scm::gl::buffer_ptr           hullvertexmap;
    scm::gl::buffer_ptr           attribute_buffer;

    scm::gl::texture_buffer_ptr   parametric_texture_buffer;
    scm::gl::texture_buffer_ptr   domain_texture_buffer;
    scm::gl::texture_buffer_ptr   obb_texture_buffer;
    scm::gl::texture_buffer_ptr   attribute_texture_buffer;
  }; 

  mutable surface_tesselation_buffer _surface_tesselation_data;
  
               
  // array and texture buffers for raycasting
  struct surface_raycasting_buffer {
    scm::gl::vertex_array_ptr     vertex_array;

    scm::gl::buffer_ptr           vertex_attrib0;
    scm::gl::buffer_ptr           vertex_attrib1;
    scm::gl::buffer_ptr           vertex_attrib2;
    scm::gl::buffer_ptr           vertex_attrib3;
    scm::gl::buffer_ptr           index_buffer;

    scm::gl::texture_buffer_ptr   controlpoints;
  };
  mutable surface_raycasting_buffer _surface_raycasting_data;

  // texture buffers for trimming   
  mutable struct {
   scm::gl::texture_buffer_ptr    partition_texture_buffer;
   scm::gl::texture_buffer_ptr    contourlist_texture_buffer;
   scm::gl::texture_buffer_ptr    curvelist_texture_buffer;
   scm::gl::texture_buffer_ptr    curvedata_texture_buffer;
   scm::gl::texture_buffer_ptr    pointdata_texture_buffer;
   scm::gl::texture_buffer_ptr    preclassification_buffer;
  } _contour_trimming_data;

  mutable scm::gl::sampler_state_ptr    _sstate;
  mutable scm::gl::rasterizer_state_ptr _rstate_no_cull;
  mutable scm::gl::rasterizer_state_ptr _rstate_cull;
  mutable scm::gl::rasterizer_state_ptr _rstate_ms_point;
  mutable scm::gl::blend_state_ptr      _bstate_no_blend;

  /////////////////////////////////////////////////////////////////////////////////////////////
  // Transformation Feedback Specific Members - only once per context
  /////////////////////////////////////////////////////////////////////////////////////////////
  struct TransformFeedbackBuffer {
    mutable scm::gl::transform_feedback_ptr _transform_feedback;
    mutable scm::gl::vertex_array_ptr       _transform_feedback_vao;
    mutable scm::gl::buffer_ptr             _transform_feedback_vbo;
  };

  /////////////////////////////////////////////////////////////////////////////////////////////

 private:  // helper methods

  void upload_to(RenderContext const& context) const;

  void initialize_states(RenderContext const& context) const;
  void initialize_texture_buffers(RenderContext const& context) const;
  void validate_texture_buffers() const;

  void initialize_transform_feedback(RenderContext const& context) const;
  void initialize_vertex_data(RenderContext const& context) const;

 private:  // attributes

  std::size_t _max_transform_feedback_buffer_size;
  mutable std::mutex upload_mutex_;
};

}  //namespace gua

#endif // GUA_NURBS_RESSOURCE_HPP
