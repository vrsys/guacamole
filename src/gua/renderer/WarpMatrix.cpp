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
#include <gua/renderer/WarpMatrix.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <fstream>

namespace gua
{
WarpMatrix::WarpMatrix() : Texture2D(0, 0), data_() {}

WarpMatrix::WarpMatrix(std::string const& file_name) : Texture2D(0, 0, scm::gl::FORMAT_RGBA_16F, 1, scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR)), data_()
{
    std::ifstream file(file_name, std::ios::binary);

    if(file)
    {
        file.read((char*)&width_, sizeof(unsigned));
        file.read((char*)&height_, sizeof(unsigned));

        data_ = std::vector<float>(width_ * height_ * 4);

        file.read((char*)data_.data(), sizeof(float) * data_.size());

        file.close();
    }
    else
    {
        Logger::LOG_WARNING << "Unable to load Warpmatrix! File " << file_name << " does not exist." << std::endl;
    }
}

void WarpMatrix::upload_to(RenderContext const& context) const
{
    std::unique_lock<std::mutex> lock(upload_mutex_);
    RenderContext::Texture ctex{};

    std::vector<void*> tmp_data;
    tmp_data.push_back(data_.data());

    ctex.texture = context.render_device->create_texture_2d(scm::gl::texture_2d_desc(scm::math::vec2ui(width_, height_), color_format_), scm::gl::FORMAT_RGBA_32F, tmp_data);

    ctex.sampler_state = context.render_device->create_sampler_state(state_descripton_);

    context.textures[uuid_] = ctex;
    context.render_context->make_resident(ctex.texture, ctex.sampler_state);
}

} // namespace gua
