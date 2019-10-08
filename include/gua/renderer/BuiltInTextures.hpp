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

#ifndef GUA_NOISE_TEXTURE_HPP
#define GUA_NOISE_TEXTURE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Texture2D.hpp>

#include <scm/gl_util/data/imaging/texture_image_data.h>

namespace gua
{
scm::gl::texture_image_data_ptr make_loading_image();
scm::gl::texture_image_data_ptr make_noise_image();
scm::gl::texture_image_data_ptr make_default_image();

} // namespace gua

#endif // GUA_NOISE_TEXTURE_HPP
