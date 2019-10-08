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

#ifndef GUA_DEPTH_CUBEMAP_RENDERER_HPP
#define GUA_DEPTH_CUBEMAP_RENDERER_HPP

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <gua/node/CubemapNode.hpp>
#include <gua/renderer/DepthCubeMap.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua
{
class Pipeline;
class PipelinePassDescription;

class DepthCubeMapRenderer
{
  public:
    void render(Pipeline& pipe, PipelinePassDescription const& desc);

    void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

    void create_state_objects(RenderContext const& ctx);

  private:
    void prepare_depth_cubemap(node::CubemapNode const& cube_map_node, Pipeline& pipe);
    void download_depth_cubemap(node::CubemapNode& cube_map_node, Pipeline const& pipe) const;

    void generate_depth_cubemap_face(unsigned face, node::CubemapNode const& cube_map_node, Pipeline& pipe) const;

    unsigned face_counter_ = 0;

    std::shared_ptr<SharedDepthCubeMapResource> depth_cube_map_res_ = nullptr;

    scm::gl::rasterizer_state_ptr rs_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_cull_none_ = nullptr;

    SubstitutionMap global_substitution_map_ = {};
    std::pair<unsigned int, std::vector<std::size_t>> needs_rendering_ = {0, std::vector<std::size_t>()};
};

} // namespace gua

#endif // GUA_DEPTH_CUBEMAP_RENDERER_HPP
