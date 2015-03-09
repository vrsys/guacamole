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

#ifndef GUA_PIPELINE_HPP
#define GUA_PIPELINE_HPP

#include <gua/node/CameraNode.hpp>
#include <gua/renderer/Renderer.hpp>
#include <gua/renderer/PipelinePass.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/LightTable.hpp>
#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/math.hpp>

#include <scm/gl_util/primitives/quad.h>

#include <memory>
#include <chrono>

namespace gua {

class WindowBase;
struct RenderContext;
class ShaderProgram;

class GUA_DLL Pipeline {
 public:

   struct GUA_DLL query_dispatch {
     scm::gl::timer_query_ptr  query;
     bool                      dispatched;
     unsigned                  collect_attempts;
   };

   struct GUA_DLL time_query_collection {
     typedef std::chrono::steady_clock::time_point   time_point;
     std::unordered_map<std::string, query_dispatch> gpu_queries;
     std::unordered_map<std::string, time_point>     cpu_queries;
     std::map<std::string, double>                   results;
   };
   
public: 

  Pipeline(RenderContext& ctx, math::vec2ui const& resolution);
  Pipeline(Pipeline const&) = delete;

  void process(CameraMode mode, node::SerializedCameraNode const& camera,
               std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs);

  std::vector<PipelinePass>   const& get_passes()  const;
  GBuffer                          & get_gbuffer() const;
  ABuffer                          & get_abuffer();
  SerializedScene                  & get_scene();
  SceneGraph                  const& get_graph()   const;
  RenderContext               const& get_context() const;
  node::SerializedCameraNode  const& get_camera()  const;
  LightTable                       & get_light_table();

  void bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const;
  void bind_light_table(std::shared_ptr<ShaderProgram> const& shader) const;
  void bind_camera_uniform_block(unsigned location) const;
  void draw_quad();
  
  // time queries
  void                                  begin_gpu_query(RenderContext const& ctx, std::string const& query_name);
  void                                  end_gpu_query(RenderContext const& ctx, std::string const& query_name);

  void                                  begin_cpu_query(std::string const& query_name);
  void                                  end_cpu_query(std::string const& query_name);

  void                                  fetch_gpu_query_results(RenderContext const& ctx);

 private:

  std::unique_ptr<GBuffer>              gbuffer_;
  ABuffer                               abuffer_;
  RenderContext&                        context_;
  std::unique_ptr<CameraUniformBlock>   camera_block_;
  std::unique_ptr<LightTable>           light_table_;

  SceneGraph const*                     current_graph_;
  SerializedScene                       current_scene_;
  node::SerializedCameraNode            current_camera_;

  math::vec2ui                          last_resolution_;
  PipelineDescription                   last_description_;
  SubstitutionMap                       global_substitution_map_;

  std::vector<PipelinePass>             passes_;
  scm::gl::quad_geometry_ptr            quad_;

#define GUA_ENABLE_PROFILING_TIME_QUERIES
  time_query_collection                 queries_;
};

}

#endif  // GUA_PIPELINE_HPP
