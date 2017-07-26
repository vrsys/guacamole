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

#ifndef GUA_TV_3_VOLUME_RENDERER_HPP
#define GUA_TV_3_VOLUME_RENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

#include <scm/gl_core/render_device/opengl/gl_core.h>
#include <gua/renderer/TV_3Renderer.hpp>
// guacamole headers
/*
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ResourceFactory.hpp>
*/
//external headers

namespace gua {

  //using 

  class MaterialShader;
  class ShaderProgram;

  class TV_3VolumeRenderer : public TV_3Renderer {
 
  public:

    TV_3VolumeRenderer(gua::RenderContext const& ctx, gua::SubstitutionMap const& substitution_map);

 private:  //shader related auxiliary methods
  
  void  _create_fbo_resources(gua::RenderContext const& ctx,
                                      scm::math::vec2ui const& render_target_dims) override;

  void  _clear_fbo_attachments(gua::RenderContext const& ctx) override;

  void  _load_shaders();

  void  _raycasting_pass(gua::Pipeline& pipe, std::vector<gua::node::Node*> const& sorted_nodes, PipelinePassDescription const& desc) override;
  void  _postprocessing_pass(gua::Pipeline& pipe, PipelinePassDescription const& desc) override;
  
 private:  //member variables

    //FBOs:
    //////////////////////////////////////////////////////////////////////////////////////
    scm::gl::frame_buffer_ptr                    volume_raycasting_fbo_;
    scm::gl::frame_buffer_ptr                    volume_compositing_fbo_;

    //accumulation pass FBO & attachments
    scm::gl::texture_2d_ptr                      volume_raycasting_back_buffer_color_result_;
    scm::gl::texture_2d_ptr                      volume_raycasting_front_buffer_color_result_;
    scm::gl::texture_2d_ptr                      volume_raycasting_back_buffer_depth_result_;
    scm::gl::texture_2d_ptr                      volume_raycasting_front_buffer_depth_result_;


    std::vector<ShaderProgramStage>              volume_compositing_shader_stages_;
    std::shared_ptr<ShaderProgram>               volume_compositing_shader_program_;

    scm::gl::blend_state_ptr volume_compositing_blend_state_;
    scm::gl::blend_state_ptr no_blending_blend_state_;
};

}

#endif  // GUA_TV_3_VOLUME_RENDERER_HPP
