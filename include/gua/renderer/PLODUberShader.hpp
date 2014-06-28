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

#ifndef GUA_PLOD_UBER_SHADER_HPP
#define GUA_PLOD_UBER_SHADER_HPP

// guacamole headers
#include <gua/renderer/GeometryUberShader.hpp>
#include <pbr/ren/cut_database_record.h>

namespace gua {

class PLODUberShader : public GeometryUberShader {

 public:

   enum pass {
     depth_pass          = 0,
     accumulation_pass   = 1,
     normalization_pass  = 2,
     reconstruction_pass = 3
   };

   enum drawing_state{
     pre_frame_state  = 0,
      pre_draw_state  = 1,
          draw_state  = 2,
     post_draw_state  = 3,
    post_frame_state  = 4,
       invalid_state  = 99999
   };

 public:

                    PLODUberShader();
                    ~PLODUberShader();

  void              create  (std::set<std::string> const& material_names);

  bool              upload_to (RenderContext const& context) const;

  /*virtual*/ stage_mask get_stage_mask() const override;

  /*virtual*/ void  preframe  (RenderContext const& context) const;

  /*virtual*/ void  predraw   (RenderContext const& ctx,
                               std::string const& filename,
                               std::string const& material_name,
                               scm::math::mat4 const& model_matrix,
                               scm::math::mat4 const& normal_matrix,
                               Frustum const& frustum,
                               View const& view) const;

  /*virtual*/ void  draw      (RenderContext const& ctx,
                              std::string const& filename,
                              std::string const& material_name,
                              scm::math::mat4 const& model_matrix,
                              scm::math::mat4 const& normal_matrix,
                              Frustum const& frustum,
                              View const& view) const;

  /*virtual*/ void  postdraw (RenderContext const& ctx,
                              std::string const& filename,
                              std::string const& material_name,
                              scm::math::mat4 const& model_matrix,
                              scm::math::mat4 const& normal_matrix,
                              Frustum const& frustum,
                              View const& view) const;

  /*virtual*/ void  postframe (RenderContext const& context) const;

 private: //auxialiary methods
  
  //get shader strings
  ///////////////////
  std::string depth_pass_vertex_shader() const;
  std::string depth_pass_fragment_shader() const;

  std::string accumulation_pass_vertex_shader() const;
  std::string accumulation_pass_fragment_shader() const;

  std::string normalization_pass_vertex_shader   () const;
  std::string normalization_pass_fragment_shader () const;

  std::string reconstruction_pass_vertex_shader () const;
  std::string reconstruction_pass_fragment_shader () const;

  std::string default_plod_material_name() const;

  //buffer operations
  char* GetMappedTempBufferPtr(RenderContext const& ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const;
  void UnmapTempBufferPtr(RenderContext const& ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const;
  void CopyTempToMainMemory(RenderContext const& ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer const&  buffer) const;

  //auxiliary
  void Reset(RenderContext const& context) const; 

 private:  //member variables

  //FBOs:
  //////////////////////////////////////////////////////////////////////////////////////
  //depth pass FBO & attachments
  mutable scm::gl::texture_2d_ptr          depth_pass_log_depth_result_;
  mutable scm::gl::texture_2d_ptr          depth_pass_linear_depth_result_;

  mutable scm::gl::frame_buffer_ptr        depth_pass_result_fbo_;

  //accumulation pass FBO & attachments
  mutable scm::gl::texture_2d_ptr          accumulation_pass_color_result_;
  mutable scm::gl::frame_buffer_ptr        accumulation_pass_result_fbo_;

  //normalization pass FBO & attachments
  mutable scm::gl::texture_2d_ptr          normalization_pass_color_result_;
  mutable scm::gl::frame_buffer_ptr        normalization_pass_result_fbo_;

  //temp buffer front & back
  //////////////////////////////////////////////////////////////////////////////////////
  mutable scm::gl::buffer_ptr              temp_buffer_A_;
  mutable scm::gl::buffer_ptr              temp_buffer_B_;

  mutable scm::gl::buffer_ptr              render_buffer_;

  mutable scm::gl::vertex_array_ptr        vertex_array_;

  mutable bool                             temp_buffer_A_is_mapped_;
  mutable bool                             temp_buffer_B_is_mapped_;
  mutable char*                            mapped_temp_buffer_A_;
  mutable char*                            mapped_temp_buffer_B_;
  mutable int                              previous_framecount_;

  //schism-GL states:
  //////////////////////////////////////////////////////////////////////////////////////
  mutable scm::gl::rasterizer_state_ptr change_point_size_in_shader_state_;

  mutable scm::gl::sampler_state_ptr       linear_sampler_state_;
  mutable scm::gl::depth_stencil_state_ptr no_depth_test_depth_stencil_state_;

  mutable scm::gl::blend_state_ptr color_accumulation_state_;

  //frustum dependent variables:
  /////////////////////////////////////////////////////////////////////////////////////
  mutable float near_plane_value_;
  mutable float height_divided_by_top_minus_bottom_;

  mutable std::vector<unsigned int>  frustum_culling_results_;

  //misc:
  ////////////////////////////////////////////////////////////////////////////////////
  mutable unsigned int material_id_;
  mutable scm::gl::quad_geometry_ptr       fullscreen_quad_;
  mutable math::vec2ui render_window_dims_;

  mutable unsigned int last_geometry_state_;
  
  mutable bool already_uploaded;


  ////////////////////////////////////////////////////////////////////////////////////
  mutable std::shared_ptr<scm::gl::context_all_guard> context_guard_;
};

}

#endif  // GUA_PLOD_UBER_SHADER_HPP
