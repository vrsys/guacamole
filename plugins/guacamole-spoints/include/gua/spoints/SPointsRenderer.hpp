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

#ifndef GUA_SPOINTS_RENDERER_HPP
#define GUA_SPOINTS_RENDERER_HPP

#include <array>

// guacamole headers
#include <gua/spoints/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <unordered_map>

namespace gua {

class ShaderProgram;
class Pipeline;
class MaterialShader;
class SPointsResource;

class GUA_SPOINTS_DLL SPointsRenderer {
 public:

  enum pass {
    warp_pass = 0,
    blend_pass = 1
  };

 public:

  SPointsRenderer();
  ~SPointsRenderer() {}

  void render(Pipeline& pipe, PipelinePassDescription const& desc);

  // /*virtual*/ void draw   (RenderContext const& context,
  //                          std::string const& ksfile_name,
  //                          std::string const& material_name,
  //                          scm::math::mat4 const& model_matrix,
  //                          scm::math::mat4 const& normal_matrix,
  //                          Frustum const& frustum,
  //                          View const& view) const;

  void set_global_substitution_map(SubstitutionMap const& smap) {
    global_substitution_map_ = smap;
  }

private:
  void          _load_shaders();
  //void          _initialize_log_to_lin_conversion_pass_program();
  void          _initialize_depth_pass_program();
  void          _initialize_accumulation_pass_program(MaterialShader* material);
  void          _initialize_normalization_pass_program();
  void          _initialize_shadow_pass_program();

  void          _initialize_forward_colored_triangles_pass_program(MaterialShader* material);
  //void          _initialize_forward_textured_triangles_pass_program(MaterialShader* material);

  std::shared_ptr<ShaderProgram> _get_material_program(MaterialShader* material,
                                                       std::shared_ptr<ShaderProgram> const& current_program,
                                                       bool& program_changed);

  void          _create_gpu_resources(gua::RenderContext const& ctx,
                                    scm::math::vec2ui const& render_target_dims,
            bool resize_resource_containers); 

 private:  // attributes

  //depth pass FBO & attachments
  scm::gl::texture_2d_ptr                      depth_pass_log_depth_result_;
  scm::gl::texture_2d_ptr                      depth_pass_linear_depth_result_;

  scm::gl::frame_buffer_ptr                    depth_pass_result_fbo_;

  //accumulation pass FBO & attachments
  scm::gl::texture_2d_ptr                      accumulation_pass_color_result_;
  scm::gl::texture_2d_ptr                      accumulation_pass_weight_and_depth_result_;
  scm::gl::frame_buffer_ptr                    accumulation_pass_result_fbo_;

  //schism-GL states:
  //////////////////////////////////////////////////////////////////////////////////////
  scm::gl::rasterizer_state_ptr                no_backface_culling_rasterizer_state_;
  scm::gl::rasterizer_state_ptr                backface_culling_rasterizer_state_;

  scm::gl::sampler_state_ptr                   nearest_sampler_state_;


  scm::gl::depth_stencil_state_ptr             no_depth_test_depth_stencil_state_;
  scm::gl::depth_stencil_state_ptr             depth_test_without_writing_depth_stencil_state_;
  scm::gl::depth_stencil_state_ptr             no_depth_test_with_writing_depth_stencil_state_;

  scm::gl::depth_stencil_state_ptr             default_depth_stencil_state_;

  scm::gl::depth_stencil_state_ptr             depth_test_with_writing_depth_stencil_state_;

  scm::gl::blend_state_ptr                     no_color_accumulation_state_;
  scm::gl::blend_state_ptr                     color_accumulation_state_;

  bool initialized_;

  bool                                         shaders_loaded_;

  scm::gl::quad_geometry_ptr                   fullscreen_quad_;

  bool                                         gpu_resources_already_created_;
  unsigned                                     previous_frame_count_;

  unsigned                                     current_rendertarget_width_;  
  unsigned                                     current_rendertarget_height_;

  mutable int last_rendered_view_id = std::numeric_limits<int>::max();
  mutable int last_rendered_side = 0;


  std::vector<ShaderProgramStage> program_stages_;

  //CPU resources
  //std::vector<ShaderProgramStage>                                      log_to_lin_conversion_shader_stages_;
  std::vector<ShaderProgramStage>                                      depth_pass_shader_stages_;
  std::vector<ShaderProgramStage>                                      accumulation_pass_shader_stages_;
  std::vector<ShaderProgramStage>                                      normalization_pass_shader_stages_;

  std::vector<ShaderProgramStage>                                      shadow_pass_shader_stages_;

  std::vector<ShaderProgramStage>                                      forward_colored_triangles_shader_stages_;


  std::vector<ShaderProgramStage>                                      forward_textured_triangles_shader_stages_;
  std::vector<ShaderProgramStage>                                      forward_textured_triangles_shader_stages_quantized_;
  //additional GPU resources 
  //std::shared_ptr<ShaderProgram>                                       log_to_lin_conversion_pass_program_;
  std::shared_ptr<ShaderProgram>                                       depth_pass_program_;
  std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram> > accumulation_pass_programs_;
  std::shared_ptr<ShaderProgram>                                       normalization_pass_program_;

  std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram> > forward_colored_triangles_pass_programs_;

  std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram> > forward_textured_triangles_pass_programs_;
  std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram> > forward_textured_triangles_pass_programs_quantized_;


  std::shared_ptr<ShaderProgram>                                       shadow_pass_program_;

  std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram> >
      programs_;
  SubstitutionMap global_substitution_map_;



  scm::gl::rasterizer_state_ptr points_rasterizer_state_;


};

}

#endif  // GUA_SPOINTS_RENDERER_HPP
