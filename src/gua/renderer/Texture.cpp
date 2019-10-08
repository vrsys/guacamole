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

namespace gua
{
Texture::Texture(scm::gl::data_format color_format, scm::gl::data_format internal_format, unsigned mipmap_layers, scm::gl::sampler_state_desc const& state_descripton)
    : mipmap_layers_(mipmap_layers), color_format_(color_format), internal_format_(internal_format), state_descripton_(state_descripton), file_name_(""), upload_mutex_()
{
}

Texture::Texture(scm::gl::data_format color_format, unsigned mipmap_layers, scm::gl::sampler_state_desc const& state_descripton)
    : mipmap_layers_(mipmap_layers), color_format_(color_format), internal_format_(color_format), state_descripton_(state_descripton), file_name_(""), upload_mutex_()
{
}

Texture::Texture(std::string const& file, bool generate_mipmaps, scm::gl::sampler_state_desc const& state_descripton)
    : mipmap_layers_(generate_mipmaps ? 1 : 0), color_format_(scm::gl::FORMAT_NULL), internal_format_(scm::gl::FORMAT_NULL), state_descripton_(state_descripton), file_name_(file), upload_mutex_()
{
}

void Texture::update_sub_data(RenderContext const& context, scm::gl::texture_region const& region, unsigned level, scm::gl::data_format format, const void* const data) const
{
    auto iter = context.textures.find(uuid_);
    if(iter == context.textures.end())
    {
        upload_to(context);
        iter = context.textures.find(uuid_);
    }

    if(iter != context.textures.end())
        context.render_context->update_sub_texture(iter->second.texture, region, level, format, data);
}

void Texture::generate_mipmaps(RenderContext const& context)
{
    auto iter = context.textures.find(uuid_);
    if(iter == context.textures.end())
    {
        upload_to(context);
        iter = context.textures.find(uuid_);
    }

    if(iter != context.textures.end())
        context.render_context->generate_mipmaps(iter->second.texture);
}

math::vec2ui const Texture::get_handle(RenderContext const& context) const
{
    auto iter = context.textures.find(uuid_);

    uint64_t handle(0);

    if(iter == context.textures.end())
    {
        upload_to(context);
        iter = context.textures.find(uuid_);
    }

    if(iter != context.textures.end())
    {
        handle = iter->second.texture->native_handle();
    }

    return math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);
}

scm::gl::texture_image_ptr const& Texture::get_buffer(RenderContext const& context) const
{
    auto iter = context.textures.find(uuid_);
    if(iter == context.textures.end())
    {
        upload_to(context);
        iter = context.textures.find(uuid_);
    }

    if(iter != context.textures.end())
    {
        return iter->second.texture;
    }
    return nullptr;
}

void Texture::make_non_resident(RenderContext const& context) const
{
    auto iter = context.textures.find(uuid_);
    if(iter != context.textures.end())
    {
        context.render_context->make_non_resident(iter->second.texture);
    }
}

} // namespace gua
