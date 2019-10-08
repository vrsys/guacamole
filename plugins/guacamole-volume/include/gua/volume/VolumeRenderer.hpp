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

#ifndef GUA_VOLUME_RENDERER_HPP
#define GUA_VOLUME_RENDERER_HPP

// guacamole headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/ResourceFactory.hpp>

namespace gua
{
class GBuffer;

class VolumeRenderer
{
  public:
    VolumeRenderer();
    ~VolumeRenderer() {}

    void render(Pipeline& pipe);

  private:
    void init_resources(Pipeline& pipe);

    scm::gl::frame_buffer_ptr volume_raygeneration_fbo_;
    std::shared_ptr<Texture2D> volume_raygeneration_color_buffer_;
    std::shared_ptr<Texture2D> volume_raygeneration_depth_buffer_;

    ResourceFactory program_factory_;
    std::shared_ptr<ShaderProgram> composite_shader_;
    std::shared_ptr<ShaderProgram> ray_generation_shader_;

    scm::gl::depth_stencil_state_ptr depth_stencil_state_;
    scm::gl::blend_state_ptr blend_state_;
};

} // namespace gua

#endif // GUA_VOLUME_RENDERER_HPP
