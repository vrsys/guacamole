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
#include <gua/renderer/Texture3D.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/math.hpp>

// external headers
#include <scm/gl_util/data/volume/volume_loader.h>
#include <iostream>

namespace gua
{
Texture3D::Texture3D(unsigned width,
                     unsigned height,
                     unsigned depth,
                     scm::gl::data_format color_format,
                     scm::gl::data_format internal_format,
                     std::vector<void*> const& data,
                     unsigned mipmap_layers,
                     scm::gl::sampler_state_desc const& state_descripton)
    : Texture(color_format, internal_format, mipmap_layers, state_descripton), width_(width), height_(height), depth_(depth), data_(data)
{
}

Texture3D::Texture3D(unsigned width, unsigned height, unsigned depth, scm::gl::data_format color_format, unsigned mipmap_layers, scm::gl::sampler_state_desc const& state_descripton)
    : Texture(color_format, mipmap_layers, state_descripton), width_(width), height_(height), depth_(depth)
{
}

Texture3D::Texture3D(std::string const& file, bool generate_mipmaps, scm::gl::sampler_state_desc const& state_descripton)
    : Texture(file, generate_mipmaps, state_descripton), width_(0), height_(0), depth_(0)
{
}

void Texture3D::upload_to(RenderContext const& context) const
{
    std::unique_lock<std::mutex> lock(upload_mutex_);
    RenderContext::Texture ctex{};

    if(file_name_ == "")
    {
        if(data_.size() == 0)
        {
            ctex.texture = context.render_device->create_texture_3d(math::vec3ui(width_, height_, depth_), color_format_, mipmap_layers_);
        }
        else
        {
            ctex.texture = context.render_device->create_texture_3d(scm::gl::texture_3d_desc(math::vec3ui(width_, height_, depth_), color_format_, mipmap_layers_), color_format_, data_);
        }
    }
    else
    {
        // MESSAGE("Uploading texture file %s", file_name_.c_str());
        scm::gl::volume_loader loader;
        ctex.texture = loader.load_texture_3d(*context.render_device, file_name_, mipmap_layers_ > 0);

        if(ctex.texture)
        {
            width_ = ctex.texture->dimensions()[0];
            height_ = ctex.texture->dimensions()[1];
            depth_ = ctex.texture->dimensions()[2];
        }
    }

    ctex.sampler_state = context.render_device->create_sampler_state(state_descripton_);

    context.textures[uuid_] = ctex;
    context.render_context->make_resident(ctex.texture, ctex.sampler_state);
}

} // namespace gua
