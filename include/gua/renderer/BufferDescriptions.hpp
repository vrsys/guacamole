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

#ifndef GUA_BUFFER_DESCRIPTIONS_HPP
#define GUA_BUFFER_DESCRIPTIONS_HPP

#include <gua/platform.hpp>

// external headers
#include <string>

namespace gua
{
/**
 * Contains information on a color buffer for a render pass.
 *
 * This struct is used to create new color buffers in render passes.
 */
struct GUA_DLL ColorBufferDescription
{
    ColorBufferDescription(std::string const& name, unsigned location, scm::gl::data_format format = scm::gl::data_format(scm::gl::FORMAT_RGB_32F)) : name_(name), location_(location), format_(format)
    {
    }

    /**
     * The name of the buffer.
     */
    std::string name_;

    /**
     * The location where it should be bound.
     */
    unsigned location_;

    /**
     * Information on the internally used data format.
     */
    scm::gl::data_format format_;
};

/**
 * Contains information on a depth stencil buffer for a render pass.
 *
 * This struct is used to create new depth stencil buffers in render passes.
 */
struct DepthStencilBufferDescription
{
    DepthStencilBufferDescription(std::string const& name, scm::gl::data_format format = scm::gl::data_format(scm::gl::FORMAT_D16)) : name_(name), format_(format) {}

    /**
     * The name of the buffer.
     */
    std::string name_;

    /**
     * Information on the internally used data format.
     */
    scm::gl::data_format format_;
};

} // namespace gua

#endif // GUA_BUFFER_DESCRIPTIONS_HPP
