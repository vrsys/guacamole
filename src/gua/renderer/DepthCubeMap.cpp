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
#include <gua/renderer/DepthCubeMap.hpp>

// guacamole headers

#include <gua/renderer/Serializer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/databases.hpp>
#include <gua/memory.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

DepthCubeMap::DepthCubeMap(RenderContext const& ctx, math::vec2ui const& resolution) :
  RenderTarget(resolution),
  viewport_offset_(math::vec2f(0.f, 0.f)),
  viewport_size_(math::vec2f(resolution)) {

  scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR,
                                    // scm::gl::FILTER_ANISOTROPIC,
                                    scm::gl::WRAP_CLAMP_TO_EDGE,
                                    scm::gl::WRAP_CLAMP_TO_EDGE);
  state._compare_mode = scm::gl::TEXCOMPARE_COMPARE_REF_TO_TEXTURE;
  state._max_anisotropy = 16;

  depth_buffer_ = std::make_shared<TextureDistance>(resolution.x, resolution.y, scm::gl::FORMAT_D16, 1, state);

  TextureDatabase::instance()->add("DepthCubeMapTestTexture", std::dynamic_pointer_cast<TextureDistance>(depth_buffer_));

  int pixel_size = viewport_size_.x * viewport_size_.y * 6; 
  int byte_size = pixel_size * sizeof(uint16_t); 

  // raw_depth_data_ = (uint16_t*)malloc(byte_size);
  world_depth_data_.reserve(pixel_size);

  fbo_ = ctx.render_device->create_frame_buffer();
  fbo_->attach_depth_stencil_buffer(depth_buffer_->get_buffer(ctx), 0, 0);
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::clear(RenderContext const& ctx, float depth, unsigned stencil) {
  ctx.render_context->clear_depth_stencil_buffer(fbo_, depth, stencil);
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::bind(RenderContext const& ctx, bool write_depth) {
  ctx.render_context->set_frame_buffer(fbo_);
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::set_viewport(RenderContext const& ctx) {
  if (ctx.render_context) {
    ctx.render_context->set_viewport(
        scm::gl::viewport(scm::math::vec2f(viewport_size_.x * viewport_offset_.x, viewport_size_.y * viewport_offset_.y),
                          scm::math::vec2f(viewport_size_)));
  }
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::set_viewport_offset(math::vec2f const& offset) {
  viewport_offset_ = offset;
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::set_viewport_size(math::vec2f const& size) {
  viewport_size_ = size;
}


////////////////////////////////////////////////////////////////////////////////
 
math::vec2f DepthCubeMap::get_viewport_size(){
    return viewport_size_;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Texture2D> const& DepthCubeMap::get_depth_buffer() const {
  return depth_buffer_;
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::remove_buffers(RenderContext const& ctx) {
  unbind(ctx);

  fbo_->clear_attachments();

  if (depth_buffer_) {
    depth_buffer_->make_non_resident(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMap::retrieve_data(RenderContext const& ctx, float near_clip, float far_clip){
  int size = (int)viewport_size_.x;

  world_depth_data_ = std::dynamic_pointer_cast<TextureDistance>(depth_buffer_)->retrieve_data(ctx, near_clip, far_clip, world_depth_data_);
  

  // ASCII OUTPUT
  // for (int side = 0; side < 6; side++){
  //   std::cout << "SIDE: " << side <<  std::endl;
  //   for (int i = 0; i<size; i++){
  //     for (int j = 0; j<size; j++){
  //       if (world_depth_data_[i*size*6 + j + side * size] == -1.0f)
  //       {
  //         std::cout << "..";
  //       }
  //       else
  //       {
  //         std::cout << "##";
  //       }
  //     }
  //     std::cout << std::endl;
  //   }
  // }
  // std::cout << "============================" << std::endl;

}

////////////////////////////////////////////////////////////////////////////////

}
