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
// guacamole headers
#include <gua/virtual_texturing/LayeredPhysicalTexture2D.hpp>


namespace gua {

  LayeredPhysicalTexture2D::LayeredPhysicalTexture2D(){}//std::string const& file,
                                                     //scm::gl::sampler_state_desc const& state_descripton) {

  //}

  LayeredPhysicalTexture2D::~LayeredPhysicalTexture2D(){
    if(!feedback_lod_cpu_buffer_) {
      delete[] feedback_lod_cpu_buffer_;
    }

    if(!feedback_count_cpu_buffer_) {
      delete[] feedback_count_cpu_buffer_;
    }    
  }

  void LayeredPhysicalTexture2D::upload_to(RenderContext const& ctx) const {
    //TODO: get_actual sizes

    width_ = 8192;
    height_ = 8192;

    scm::math::vec2ui physical_texture_dimensions(width_, height_);

    num_layers_ = 5;

    uint32_t tile_size = 256;

    num_feedback_slots_ = (width_ / tile_size) * (height_ / tile_size) * num_layers_;




    physical_texture_ptr_ = ctx.render_device->create_texture_2d(physical_texture_dimensions, scm::gl::FORMAT_RGB_8, 1,
                                                                 num_layers_ + 1);

    feedback_lod_storage_ = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STREAM_COPY,
                                                             num_feedback_slots_ * size_of_format(scm::gl::FORMAT_R_32I));

    feedback_count_storage_  = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STREAM_COPY,
                                                                num_feedback_slots_ * size_of_format(scm::gl::FORMAT_R_32UI));

    
    ctx.render_context->bind_storage_buffer(feedback_lod_storage_, 0);
    ctx.render_context->bind_storage_buffer(feedback_count_storage_, 1);



    feedback_lod_cpu_buffer_   = new int32_t[num_feedback_slots_];
    feedback_count_cpu_buffer_ = new uint32_t[num_feedback_slots_];



    std::cout << "Creating Physical Texture\n";
  }

}  
