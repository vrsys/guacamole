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
// guacamole headers
#include <gua/virtual_texturing/LayeredPhysicalTexture2D.hpp>


namespace gua {

  LayeredPhysicalTexture2D::LayeredPhysicalTexture2D(){}//std::string const& file,
                                                     //scm::gl::sampler_state_desc const& state_descripton) {

  //}



  void LayeredPhysicalTexture2D::upload_to(RenderContext const& ctx) const {
    //TODO: get_actual sizes
    scm::math::vec2ui const physical_texture_dimensions = scm::math::vec2ui(8192, 8192);
    uint32_t const num_physical_texture_layers = 5;

    physical_texture_ptr_ = ctx.render_device->create_texture_2d(physical_texture_dimensions, scm::gl::FORMAT_RGB_8, 1,
                                                                 num_physical_texture_layers + 1);

    std::cout << "Creating Physical Texture\n";
  }

}  
