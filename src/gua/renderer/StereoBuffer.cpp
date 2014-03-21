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
#include <gua/renderer/StereoBuffer.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

StereoBuffer::StereoBuffer(
    RenderContext const& ctx,
    Pipeline::Configuration const& config,
    std::vector<std::pair<BufferComponent, scm::gl::sampler_state_desc> > const&
        layers) {

    if (!config.get_enable_stereo()) {
        eye_buffers_.push_back(new GBuffer(layers,
                                           config.get_left_resolution()[0],
                                           config.get_left_resolution()[1]));
    } else {
        eye_buffers_.push_back(new GBuffer(layers,
                                           config.get_left_resolution()[0],
                                           config.get_left_resolution()[1]));
        eye_buffers_.push_back(new GBuffer(layers,
                                           config.get_right_resolution()[0],
                                           config.get_right_resolution()[1]));
    }

    for (auto buffer : eye_buffers_) {
        buffer->create(ctx);
    }
}

////////////////////////////////////////////////////////////////////////////////

StereoBuffer::~StereoBuffer() {
    for (auto buffer : eye_buffers_) {
        delete buffer;
    }
}

void StereoBuffer::remove_buffers(RenderContext const& ctx) {
    for (auto buffer : eye_buffers_) {
        buffer->remove_buffers(ctx);
    }
}

////////////////////////////////////////////////////////////////////////////////

void StereoBuffer::clear(RenderContext const & ctx) {
    for (auto buffer : eye_buffers_) {
        buffer->clear_color_buffers(ctx);
        buffer->clear_depth_stencil_buffer(ctx);
    }
}

////////////////////////////////////////////////////////////////////////////////

std::vector<GBuffer*> StereoBuffer::get_eye_buffers() const {

    return eye_buffers_;
}

////////////////////////////////////////////////////////////////////////////////

}
