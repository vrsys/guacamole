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

// class header
#include <gua/renderer/OcclusionSlaveResolvePass.hpp>

#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
OcclusionSlaveResolvePassDescription::OcclusionSlaveResolvePassDescription()
  : PipelinePassDescription()
{
  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = "shaders/resolve.frag";
  name_ = "OcclusionSlaveResolvePass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Quad;
  depth_stencil_state_ = boost::make_optional(
    scm::gl::depth_stencil_state_desc(
      false, false, scm::gl::COMPARISON_LESS, true, 1, 0,
      scm::gl::stencil_ops(scm::gl::COMPARISON_EQUAL)
    )
  );

}

void OcclusionSlaveResolvePassDescription::apply_post_render_action(RenderContext const& ctx, gua::Pipeline* pipe) const {


    //pipe->get_gbuffer()->toggle_ping_pong();
    auto& target = pipe->current_viewstate().target;




    auto& gua_depth_buffer = target->get_depth_buffer();

    //gua_depth_buffer->toggle_ping_pong();


  auto const& camera = pipe->current_viewstate().camera;



  scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();



  uint32_t pixel_size = render_target_dims[0] * render_target_dims[1];
 // uint32_t byte_size = pixel_size * sizeof(uint32_t); 
  //texture_data_ = (uint32_t*)malloc(byte_size);
  //world_depth_data_ = std::vector<float>(pixel_size, -1.0f);i0
  std::vector<uint32_t> texture_data(pixel_size, 0);

  pipe->get_gbuffer()->retrieve_depth_data(ctx, (uint32_t*)&texture_data[0]);

  //ctx.render_context->retrieve_texture_data(gua_depth_buffer, 0, (void*)&texture_data[0]);
   // pipe->get_gbuffer()->toggle_ping_pong();


/*
  std::cout << "Retrieving Depth buffer data!\n";

  uint32_t max_uint = std::numeric_limits<uint32_t>::max();

  for(int y_idx = 0; y_idx < render_target_dims[1]; y_idx += (render_target_dims[1]) / 20 ) {
    for(int x_idx = 0; x_idx < render_target_dims[0]; x_idx += (render_target_dims[0]) / 20) {

      float normalized_depth = (texture_data[x_idx + y_idx * render_target_dims[0]] ) / (float)(max_uint);
      printf("%.10f ", normalized_depth);
    }
    std::cout << "\n";
  }
  std::cout << "\n";
*/


/*
  TextureDistance::TextureDistance(unsigned width,
                   unsigned height,
                   scm::gl::data_format color_format,
                   unsigned mipmap_layers,
                   scm::gl::sampler_state_desc const& state_descripton)
    : Texture2D(width, height, color_format, mipmap_layers, state_descripton){

    int pixel_size = height_ * width_ * 6; 
    int byte_size = pixel_size * sizeof(uint32_t); 
    texture_data_ = (uint32_t*)malloc(byte_size);
    world_depth_data_ = std::vector<float>(pixel_size, -1.0f);
  }



  RenderContext const& ctx(pipe.get_context());
  auto& target = *pipe.current_viewstate().target;
  auto const& camera = pipe.current_viewstate().camera;

  scm::math::vec2ui const& render_target_dims = camera.config.get_resolution();


  scm::gl::context_all_guard context_guard(ctx.render_context);
  auto& gua_depth_buffer = target.get_depth_buffer();

get_depth_buffer()
    ctx.render_context->retrieve_texture_data(get_buffer(ctx), 0, texture_data_);
    */
/*    unsigned size = height_*width_;
    for (int texel = 0; texel < size; ++texel){
      if (texture_data_[texel] == 0xFFFFFFFF){
        world_depth_data_[texel] = -1.0;
      }else{
        float x = (float(texel%height_) / height_) - 0.5; //height = width/6
        float y = (float(texel/width_) / height_) - 0.5;
        float z = (float)texture_data_[texel] / 4294967295.0;

        float z_n = 2.0 * near_clip * far_clip / (far_clip + near_clip - z * (far_clip - near_clip));
        float x_n = x * z_n / 0.5;
        float y_n = y * z_n / 0.5;

        float distance = std::sqrt( (x_n*x_n) + (y_n*y_n) + (z_n*z_n) ) * 0.5;
        world_depth_data_[texel] = distance;
      }
    } 
  }
*/






}


////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> OcclusionSlaveResolvePassDescription::make_copy() const {
  return std::make_shared<OcclusionSlaveResolvePassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass OcclusionSlaveResolvePassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) {


  PipelinePass pass{*this, ctx, substitution_map};
  return pass;
}

}
