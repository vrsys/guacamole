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

#ifndef GUA_SKYMAP_RENDERER_HPP
#define GUA_SKYMAP_RENDERER_HPP

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/TextureCube.hpp>
#include <gua/renderer/Pipeline.hpp>

namespace gua
{
class SkyMapRenderer
{
  public:
    SkyMapRenderer();
    virtual ~SkyMapRenderer() {}

    void render_sky_map(Pipeline& pipe, PipelinePassDescription const& desc);

  private:
    std::shared_ptr<ShaderProgram> program_;
    std::shared_ptr<TextureCube> sky_map_;
    scm::gl::frame_buffer_ptr fbo_;
};

} // namespace gua

#endif // GUA_SKYMAP_RENDERER_HPP
