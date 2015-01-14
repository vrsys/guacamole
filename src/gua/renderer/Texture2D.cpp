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
#include <gua/renderer/Texture2D.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/math.hpp>

// external headers
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <iostream>

namespace gua {

Texture2D::Texture2D(unsigned width,
                 unsigned height,
                 scm::gl::data_format color_format,
                 scm::gl::data_format internal_format,
                 std::vector<void*> const& data,
                 unsigned mipmap_layers,
                 scm::gl::sampler_state_desc const& state_descripton)
    : Texture(color_format, internal_format, data, mipmap_layers, state_descripton),
      width_(width),
      height_(height) {}

Texture2D::Texture2D(unsigned width,
                 unsigned height,
                 scm::gl::data_format color_format,
                 unsigned mipmap_layers,
                 scm::gl::sampler_state_desc const& state_descripton)
    : Texture(color_format, mipmap_layers, state_descripton),
      width_(width),
      height_(height) {}

Texture2D::Texture2D(std::string const& file,
                 bool generate_mipmaps,
                 scm::gl::sampler_state_desc const& state_descripton)
    : Texture(file, generate_mipmaps, state_descripton),
      width_(0),
      height_(0) {}

void Texture2D::upload_to(RenderContext const& context) const {

  std::unique_lock<std::mutex> lock(upload_mutex_);

  if (textures_.size() <= context.id) {
    textures_.resize(context.id + 1);
    sampler_states_.resize(context.id + 1);
    render_contexts_.resize(context.id + 1);
  }

  if (file_name_ == "") {

    if (data_.size() == 0)
      textures_[context.id] = context.render_device->create_texture_2d(
          math::vec2ui(width_, height_), color_format_, mipmap_layers_);
    else
      textures_[context.id] = context.render_device->create_texture_2d(
          scm::gl::texture_2d_desc(
              math::vec2ui(width_, height_), color_format_, mipmap_layers_
          ), internal_format_, data_);
  } else {
    scm::gl::texture_loader loader;
    textures_[context.id] = loader.load_texture_2d(
        *context.render_device, file_name_, mipmap_layers_ > 0);

    if (textures_[context.id]) {
      width_ = textures_[context.id]->dimensions()[0];
      height_ = textures_[context.id]->dimensions()[1];
    }
  }

  if (textures_[context.id]) {
    sampler_states_[context.id] =
        context.render_device->create_sampler_state(state_descripton_);

    render_contexts_[context.id] = context.render_context;

    make_resident(context);
  }
}

std::vector<void*>& Texture2D::get_data() {
  return data_;
}

}
