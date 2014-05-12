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

   enum pass {
     depth_pass          = 0,
     accumulation_pass   = 1,
     normalization_pass  = 2
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

                    PBRUberShader();

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
  std::string const depth_pass_vertex_shader() const;
  std::string const depth_pass_fragment_shader() const;

  std::string const accumulation_pass_vertex_shader() const;
  std::string const accumulation_pass_fragment_shader() const;

  std::string const normalization_pass_vertex_shader   () const;
  std::string const normalization_pass_fragment_shader () const;

  std::string const default_pbr_material_name() const;

 private:  //member variables

  //FBOs:
  //////////////////////////////////////////////////////////////////////////////////////
  //depth pass FBO & attachments
  mutable std::vector<scm::gl::texture_2d_ptr>	        depth_pass_log_depth_result_;
  mutable std::vector<scm::gl::texture_2d_ptr>	        depth_pass_linear_depth_result_;

  mutable std::vector<scm::gl::frame_buffer_ptr>        depth_pass_result_fbo_;

  //accumulation pass FBO & attachments
  mutable std::vector<scm::gl::texture_2d_ptr>	        accumulation_pass_color_result_;
  mutable std::vector<scm::gl::frame_buffer_ptr>        accumulation_pass_result_fbo_;


  //schism-GL states:
  //////////////////////////////////////////////////////////////////////////////////////
  mutable std::vector<scm::gl::rasterizer_state_ptr> change_point_size_in_shader_state_;

  mutable std::vector<scm::gl::sampler_state_ptr>       linear_sampler_state_;
  mutable std::vector<scm::gl::depth_stencil_state_ptr> no_depth_test_depth_stencil_state_;

  mutable std::vector<scm::gl::blend_state_ptr> color_accumulation_state_;
  

  
  //frustum dependent variables:
  /////////////////////////////////////////////////////////////////////////////////////
  mutable std::vector<float> near_plane_value_;
  mutable std::vector<float> height_divided_by_top_minus_bottom_;

  //misc:
  ////////////////////////////////////////////////////////////////////////////////////
  mutable std::vector<unsigned int> material_id_;
  mutable std::vector<scm::gl::quad_geometry_ptr>       fullscreen_quad_;
  mutable std::vector<math::vec2ui> render_window_dims_;

  mutable std::vector<unsigned int> last_geometry_state_;


  ////////////////////////////////////////////////////////////////////////////////////
  mutable std::vector<std::shared_ptr<scm::gl::context_all_guard>> context_guard_;

  mutable std::vector<unsigned int> framecount_ ;
};

}

#endif  // GUA_PBR_UBER_SHADER_HPP
