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

namespace gua
{
LayeredPhysicalTexture2D::LayeredPhysicalTexture2D() {}

LayeredPhysicalTexture2D::~LayeredPhysicalTexture2D()
{
    if(!feedback_lod_cpu_buffer_)
    {
        delete[] feedback_lod_cpu_buffer_;
    }

    if(!feedback_count_cpu_buffer_)
    {
        delete[] feedback_count_cpu_buffer_;
    }
}

void LayeredPhysicalTexture2D::upload_to(RenderContext const& ctx, uint32_t width, uint32_t height, uint32_t num_layers, uint32_t tile_size) const
{
    width_ = width;
    height_ = height;
    num_layers_ = num_layers;
    tile_size_ = tile_size;
    num_feedback_slots_ = (width_ / tile_size_) * (height_ / tile_size_) * num_layers_;

    scm::math::vec2ui physical_texture_dimensions(width_, height_);

    auto phys_tex_format = scm::gl::FORMAT_RGBA_8;
    switch(::vt::VTConfig::get_instance().get_format_texture())
    {
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

    physical_texture_ptr_ = ctx.render_device->create_texture_2d(physical_texture_dimensions, phys_tex_format, 1, num_layers_ + 1);

    auto linear_sampler_state = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

    ctx.render_context->make_resident(physical_texture_ptr_, linear_sampler_state);
    feedback_lod_storage_ = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STREAM_COPY, num_feedback_slots_ * size_of_format(scm::gl::FORMAT_R_32I));

    feedback_count_storage_ = ctx.render_device->create_buffer(scm::gl::BIND_STORAGE_BUFFER, scm::gl::USAGE_STREAM_COPY, num_feedback_slots_ * size_of_format(scm::gl::FORMAT_R_32UI));

    physical_texture_address_ubo_ = ctx.render_device->create_buffer(scm::gl::BIND_UNIFORM_BUFFER, scm::gl::USAGE_STATIC_DRAW, 4 * sizeof(scm::math::vec2ui));

    ctx.render_context->bind_storage_buffer(feedback_lod_storage_, 0);
    ctx.render_context->bind_storage_buffer(feedback_count_storage_, 1);

    upload_physical_texture_handle_to_ubo(ctx);

    feedback_lod_cpu_buffer_ = new int32_t[num_feedback_slots_];
    feedback_count_cpu_buffer_ = new uint32_t[num_feedback_slots_];
}

void LayeredPhysicalTexture2D::upload_physical_texture_handle_to_ubo(RenderContext const& ctx) const
{
    uint64_t handle = physical_texture_ptr_->native_handle();

    // math::vec2ui swapped_texture_adress = math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);

    uint64_t* mapped_physical_texture_address_ubo = (uint64_t*)ctx.render_context->map_buffer(physical_texture_address_ubo_, scm::gl::ACCESS_WRITE_ONLY);

    uint64_t copy_byte_offset_write = 0;
    uint64_t physical_texture_cpu_address = (handle & 0x00000000ffffffff) | (handle & 0xffffffff00000000);
    memcpy((char*)(&mapped_physical_texture_address_ubo[0]) + copy_byte_offset_write, &physical_texture_cpu_address, sizeof(uint64_t));
    copy_byte_offset_write += sizeof(uint64_t);

    scm::math::vec2ui tile_size{tile_size_, tile_size_};
    memcpy((char*)(&mapped_physical_texture_address_ubo[0]) + copy_byte_offset_write, &tile_size, sizeof(uint64_t));
    copy_byte_offset_write += sizeof(scm::math::vec2ui);

    scm::math::vec2f tile_padding{1.0f, 1.0};
    memcpy((char*)(&mapped_physical_texture_address_ubo[0]) + copy_byte_offset_write, &tile_padding, sizeof(uint64_t));
    copy_byte_offset_write += sizeof(scm::math::vec2f);

    scm::math::vec2ui physical_texture_dim_tiles{width_ / tile_size[0], width_ / tile_size[1]};
    memcpy((char*)(&mapped_physical_texture_address_ubo[0]) + copy_byte_offset_write, &physical_texture_dim_tiles, sizeof(uint64_t));
    copy_byte_offset_write += sizeof(scm::math::vec2ui);

    ctx.render_context->unmap_buffer(physical_texture_address_ubo_);

    ctx.render_context->bind_uniform_buffer(physical_texture_address_ubo_, 3);
}
} // namespace gua
