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
#include <gua/renderer/DepthCubeMapRenderer.hpp>

#include <gua/config.hpp>
#include <gua/node/CubemapNode.hpp>

#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/DepthCubeMap.hpp>
#include <gua/databases/Resources.hpp>

#include <scm/core/math/math.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

DepthCubeMapRenderer::DepthCubeMapRenderer()
  : mode_(DepthCubeMapRenderer::COMPLETE),
    face_counter_(0)
{
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMapRenderer::create_state_objects(RenderContext const& ctx)
{
  rs_cull_back_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_BACK);
  rs_cull_none_ = ctx.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);
}

////////////////////////////////////////////////////////////////////////////////

void DepthCubeMapRenderer::render(Pipeline& pipe, PipelinePassDescription const& desc)
{

  auto& scene = *pipe.current_viewstate().scene;
  auto cube_map_nodes(scene.nodes.find(std::type_index(typeid(node::CubemapNode))));
  
  if (cube_map_nodes != scene.nodes.end()) {

    RenderContext const& ctx(pipe.get_context());
  
    std::string const gpu_query_name = "GPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / DepthCubeMapPass";
    std::string const cpu_query_name = "CPU: Camera uuid: " + std::to_string(pipe.current_viewstate().viewpoint_uuid) + " / DepthCubeMapPass";

    pipe.begin_gpu_query(ctx, gpu_query_name);
    pipe.begin_cpu_query(cpu_query_name);

    for (auto const& object : cube_map_nodes->second){
      auto cube_map_node(reinterpret_cast<node::CubemapNode*>(object));
      if (mode_ == DepthCubeMapRenderer::COMPLETE){
        pipe.prepare_depth_cubemap(cube_map_node);
        pipe.generate_depth_cubemap_face(0, cube_map_node);
        pipe.generate_depth_cubemap_face(0, cube_map_node);
        pipe.generate_depth_cubemap_face(1, cube_map_node);
        pipe.generate_depth_cubemap_face(2, cube_map_node);
        pipe.generate_depth_cubemap_face(3, cube_map_node);
        pipe.generate_depth_cubemap_face(4, cube_map_node);
        pipe.generate_depth_cubemap_face(5, cube_map_node);
        pipe.reset_depth_cubemap(cube_map_node);
      } else if (mode_ == DepthCubeMapRenderer::ONE_SIDE_PER_FRAME) {
        if (face_counter_ == 0){
          pipe.reset_depth_cubemap(cube_map_node);
        }
        pipe.generate_depth_cubemap_face(face_counter_, cube_map_node);
        face_counter_ = (face_counter_+1)%6;
      }
    }

    pipe.end_gpu_query(ctx, gpu_query_name);
    pipe.end_cpu_query(cpu_query_name);

  }
  // ctx.render_context->reset_state_objects();


}

////////////////////////////////////////////////////////////////////////////////

} // namespace gua
