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
#include <gua/renderer/GBuffer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/logger.hpp>

namespace gua {

GBuffer::GBuffer(
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> > const&
        layers,
    unsigned width,
    unsigned height,
    unsigned mipmap_layers)
    : layer_types_(layers),
      width_(width),
      height_(height),
      mipmap_layers_(mipmap_layers) {

    color_buffers_[TYPE_INTEGER] = std::vector<std::shared_ptr<Texture> >();
    color_buffers_[TYPE_UNSIGNED] = std::vector<std::shared_ptr<Texture> >();
    color_buffers_[TYPE_HALF] = std::vector<std::shared_ptr<Texture> >();
    color_buffers_[TYPE_FLOAT] = std::vector<std::shared_ptr<Texture> >();
}

void GBuffer::remove_buffers(RenderContext const& ctx) {
  unbind(ctx);
  remove_attachments();

  if (depth_buffer_)
    depth_buffer_->make_non_resident(ctx);

  for (auto const& c: color_buffers_)
    for (auto b: c.second)
      b->make_non_resident(ctx);
}

void GBuffer::create(RenderContext const& ctx) {

    int color_attachment_count(0);
    for (auto const& layer : layer_types_) {
        BufferComponent type(layer.first);
        scm::gl::sampler_state_desc const& state(layer.second);

        switch (type) {
            case BufferComponent::I1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_R_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::I2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::I3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::I4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>(width_,
                                             height_,
                                             scm::gl::FORMAT_R_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>(width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>(width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_R_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16F,
                                             mipmap_layers_,
                                             state));
              break;


            ///TODO: Make it work with 32bit
            case BufferComponent::F1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_R_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::DEPTH_16:
              depth_buffer_ = std::make_shared<Texture>(
                  width_, height_, scm::gl::FORMAT_D16, mipmap_layers_, state);
              break;
            case BufferComponent::DEPTH_24:
              depth_buffer_ = std::make_shared<Texture>(
                  width_, height_, scm::gl::FORMAT_D24, mipmap_layers_, state);
              break;
            default:
              ;
        }

        if (type == BufferComponent::DEPTH_16 || type == BufferComponent::DEPTH_24)
            attach_depth_stencil_buffer(ctx, depth_buffer_);
        else if (type != BufferComponent::NONE)
            attach_color_buffer(
                ctx,
                color_attachment_count++,
                *color_buffers_[enums::get_type(type)].rbegin());
    }
}

void GBuffer::create_UGLY(RenderContext const & ctx) {

    int color_attachment_count(0);

    for (auto const& layer : layer_types_) {
        BufferComponent type(layer.first);
        scm::gl::sampler_state_desc const& state(layer.second);
        switch (type) {
            case BufferComponent::I1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>(width_,
                                             height_,
                                             scm::gl::FORMAT_R_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::I2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>(width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::I3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>(width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::I4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16I,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_R_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::U4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16UI,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_R_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RG_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::H4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_16F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F1:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_R_32F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F2:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RG_32F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F3:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGB_32F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::F4:
              color_buffers_[enums::get_type(type)].push_back(
                  std::make_shared<Texture>( width_,
                                             height_,
                                             scm::gl::FORMAT_RGBA_32F,
                                             mipmap_layers_,
                                             state));
              break;
            case BufferComponent::DEPTH_16:
              depth_buffer_ = std::make_shared<Texture>(
                  width_, height_, scm::gl::FORMAT_D16, mipmap_layers_, state);
              break;
            case BufferComponent::DEPTH_24:
              depth_buffer_ = std::make_shared<Texture>(
                  width_, height_, scm::gl::FORMAT_D24, mipmap_layers_, state);
              break;
            default:
              ;
        }

        if (type == BufferComponent::DEPTH_16 || type == BufferComponent::DEPTH_24)
            attach_depth_stencil_buffer(ctx, depth_buffer_);
        else if (type != BufferComponent::NONE)
            attach_color_buffer(
                ctx,
                color_attachment_count++,
                *color_buffers_[enums::get_type(type)].rbegin());
    }
}

std::vector<std::shared_ptr<Texture> > const& GBuffer::get_color_buffers(
    BufferComponentType type) const {

    return color_buffers_.find(type)->second;
}

std::shared_ptr<Texture> const& GBuffer::get_depth_buffer() const {
    return depth_buffer_;
}

}
