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

// lamure headers
#include <lamure/vt/VTConfig.h>

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

  void LayeredPhysicalTexture2D::upload_to(RenderContext const& ctx, uint32_t width, uint32_t height, uint32_t num_layers, uint32_t tile_size) const {
    //TODO: get_actual sizes

    width_ = width;
    height_ = height;
    num_layers_ = num_layers;
    tile_size_ = tile_size;
    num_feedback_slots_ = (width_ / tile_size_) * (height_ / tile_size_) * num_layers_;
    //auto physical_texture_size = scm::math::vec2ui(),
    //                                               );


    scm::math::vec2ui physical_texture_dimensions(width_, height_);



    auto phys_tex_format = scm::gl::FORMAT_RGBA_8;
    switch (::vt::VTConfig::get_instance().get_format_texture()) {
        case ::vt::VTConfig::R8:
            phys_tex_format = scm::gl::FORMAT_R_8;
            break;
        case ::vt::VTConfig::RGB8:
            phys_tex_format = scm::gl::FORMAT_RGB_8;
            break;
        case ::vt::VTConfig::RGBA8:
        default:
            phys_tex_format = scm::gl::FORMAT_RGBA_8;
            break;
    }




    physical_texture_ptr_ = ctx.render_device->create_texture_2d(physical_texture_dimensions, phys_tex_format, 1,
                                                                 num_layers_ + 1);

    nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
    linear_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

    ctx.render_context->make_resident(physical_texture_ptr_, linear_sampler_state_);

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

  math::vec2ui LayeredPhysicalTexture2D::get_physical_texture_handle(RenderContext const& ctx) const {
    uint64_t handle = physical_texture_ptr_->native_handle();
    return math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
  }

  void LayeredPhysicalTexture2D::upload_physical_texture_handle(RenderContext const& ctx) const {
    
  }

}  
