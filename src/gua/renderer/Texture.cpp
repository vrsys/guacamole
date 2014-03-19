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
#include <gua/renderer/Texture.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/math.hpp>

// external headers
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <iostream>

namespace gua {

Texture::Texture(scm::gl::data_format color_format,
                 std::vector<void*> const& data,
                 unsigned mipmap_layers,
                 scm::gl::sampler_state_desc const& state_descripton)
    : mipmap_layers_(mipmap_layers),
      color_format_(color_format),
      file_name_(""),
      data_(data),
      state_descripton_(state_descripton),
      textures_(),
      sampler_states_(),
      upload_mutex_() {}

Texture::Texture(scm::gl::data_format color_format,
                 unsigned mipmap_layers,
                 scm::gl::sampler_state_desc const& state_descripton)
    : mipmap_layers_(mipmap_layers),
      color_format_(color_format),
      file_name_(""),
      state_descripton_(state_descripton),
      textures_(),
      sampler_states_(),
      upload_mutex_() {}

Texture::Texture(std::string const& file,
                 bool generate_mipmaps,
                 scm::gl::sampler_state_desc const& state_descripton)
    : 
      mipmap_layers_(generate_mipmaps ? 1 : 0),
      color_format_(scm::gl::FORMAT_NULL),
      file_name_(file),
      state_descripton_(state_descripton),
      textures_(),
      sampler_states_(),
      upload_mutex_() {}

Texture::~Texture() {
  make_non_resident();
}

void Texture::generate_mipmaps(RenderContext const& context) {

  if (textures_.size() <= context.id || textures_[context.id] == 0)
    upload_to(context);

  context.render_context->generate_mipmaps(textures_[context.id]);
}

math::vec2ui const Texture::get_handle(RenderContext const& context) const {

  if (textures_.size() <= context.id || textures_[context.id] == 0)
    upload_to(context);

  uint64_t handle(textures_[context.id]->native_handle());

  return math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

scm::gl::texture_image_ptr const& Texture::get_buffer(
    RenderContext const& context) const {

  if (textures_.size() <= context.id || textures_[context.id] == 0)
    upload_to(context);

  return textures_[context.id];
}

void Texture::make_resident(RenderContext const& context) const {
  context.render_context
      ->make_resident(textures_[context.id], sampler_states_[context.id]);

}

void Texture::make_non_resident(RenderContext const& context) const {
  context.render_context->make_non_resident(textures_[context.id]);
}

void Texture::make_non_resident() const {
  for (int i(0); i<textures_.size(); ++i ) {
    render_contexts_[i]->make_non_resident(textures_[i]);
  }
}

#if 0
void Texture::upload_to(RenderContext const& context) const {
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
              math::vec2ui(width_, height_), color_format_, mipmap_layers_),
          color_format_,
          data_);
  } else {
    MESSAGE("Uploading texture file %s", file_name_.c_str());
    scm::gl::texture_loader loader;
    textures_[context.id] = loader.load_texture_2d(
        *context.render_device, file_name_, mipmap_layers_ > 0);

    if (textures_[context.id]) {
      width_ = textures_[context.id]->dimensions()[0];
      height_ = textures_[context.id]->dimensions()[1];
    }
  }

  sampler_states_[context.id] =
      context.render_device->create_sampler_state(state_descripton_);

  render_contexts_[context.id] = context.render_context;

  make_resident(context);
}
#endif

}
