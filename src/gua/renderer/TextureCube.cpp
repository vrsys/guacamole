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
#include <gua/renderer/TextureCube.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/math.hpp>

// external headers
#include <scm/gl_util/data/imaging/texture_loader.h>
#include <iostream>

namespace gua
{
TextureCube::TextureCube(
    unsigned width, unsigned height, scm::gl::data_format color_format, scm::gl::data_format internal_format, unsigned mipmap_layers, scm::gl::sampler_state_desc const& state_descripton)
    : Texture(color_format, internal_format, mipmap_layers, state_descripton), width_(width), height_(height), file_px_(""), file_nx_(""), file_py_(""), file_ny_(""), file_pz_(""), file_nz_("")
{
}

TextureCube::TextureCube(unsigned width, unsigned height, scm::gl::data_format color_format, unsigned mipmap_layers, scm::gl::sampler_state_desc const& state_descripton)
    : Texture(color_format, mipmap_layers, state_descripton), width_(width), height_(height), file_px_(""), file_nx_(""), file_py_(""), file_ny_(""), file_pz_(""), file_nz_("")
{
}

TextureCube::TextureCube(std::string const& file_px,
                         std::string const& file_nx,
                         std::string const& file_py,
                         std::string const& file_ny,
                         std::string const& file_pz,
                         std::string const& file_nz,
                         bool generate_mipmaps,
                         scm::gl::sampler_state_desc const& state_descripton)
    : Texture(file_px, generate_mipmaps, state_descripton), width_(0), height_(0), file_px_(file_px), file_nx_(file_nx), file_py_(file_py), file_ny_(file_ny), file_pz_(file_pz), file_nz_(file_nz)
{
}

void TextureCube::upload_to(RenderContext const& context) const
{
    std::unique_lock<std::mutex> lock(upload_mutex_);
    RenderContext::Texture ctex{};

    if(file_name_ == "")
    {
        // if (data_.size() == 0) {
        ctex.texture = context.render_device->create_texture_cube(math::vec2ui(width_, height_), color_format_, mipmap_layers_);
        // } else {
        //   ctex.texture = context.render_device->create_texture_cube(
        //       scm::gl::texture_cube_desc(
        //           math::vec2ui(width_, height_), color_format_, mipmap_layers_
        //       ), internal_format_, data_, data_, data_, data_, data_, data_);
        // }
    }
    else
    {
        scm::gl::texture_loader loader;
        ctex.texture = loader.load_texture_cube(*context.render_device, file_px_, file_nx_, file_py_, file_ny_, file_pz_, file_nz_, mipmap_layers_ > 0);

        if(ctex.texture)
        {
            width_ = ctex.texture->dimensions()[0];
            height_ = ctex.texture->dimensions()[1];
        }
    }

    if(ctex.texture)
    {
        ctex.sampler_state = context.render_device->create_sampler_state(state_descripton_);

        context.textures[uuid_] = ctex;
        context.render_context->make_resident(ctex.texture, ctex.sampler_state);
    }
}

} // namespace gua
