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
#include <gua/renderer/ShadowMap.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/LightTable.hpp>
#include <gua/renderer/LightTransformationUniformBlock.hpp>
#include <gua/renderer/CameraUniformBlock.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/math.hpp>

#include <scm/gl_util/primitives/quad.h>

#include <memory>
#include <chrono>

namespace gua
{
class WindowBase;
struct RenderContext;
class ShaderProgram;

struct GUA_DLL PipelineViewState
{
    enum ViewDirection
    {
        front = 0,
        back = 1,
        left = 2,
        right = 3,
        top = 4,
        bottom = 5,
        count = 6
    };

    PipelineViewState() = default;

    RenderTarget* target = nullptr;

    SceneGraph const* graph = nullptr;
    std::shared_ptr<SerializedScene> scene = nullptr;
    node::SerializedCameraNode camera;

    Frustum frustum;
    std::size_t viewpoint_uuid = 0;

    ViewDirection view_direction = front;
    bool shadow_mode = false;
};


class GUA_DLL Pipeline
{
  public:
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    struct GUA_DLL query_dispatch
    {
        scm::gl::timer_query_ptr query;
        bool dispatched;
        unsigned collect_attempts;
    };

    struct GUA_DLL time_query_collection
    {
        using time_point = std::chrono::steady_clock::time_point;
        std::unordered_map<std::string, query_dispatch> gpu_queries;
        std::unordered_map<std::string, time_point> cpu_queries;
        std::map<std::string, double> results;
    };
#endif

  public:
    friend class LightVisibilityRenderer;
    friend class DepthCubeMapRenderer;

    Pipeline(RenderContext& ctx, math::vec2ui const& resolution);
    Pipeline(Pipeline const&) = delete;

    scm::gl::texture_2d_ptr render_scene(CameraMode mode, node::SerializedCameraNode const& camera, std::vector<std::unique_ptr<const SceneGraph>> const& scene_graphs, bool render_multiview, bool use_hardware_mvr);

    void load_passes_and_responsibilities();

    void fulfil_pre_render_responsibilities(RenderContext const& ctx);
    void fulfil_post_render_responsibilities(RenderContext const& ctx);

    void generate_shadow_map(node::LightNode& light, LightTable::LightBlock& light_block);

    PipelineViewState const& current_viewstate() const;

    RenderContext& get_context();
    RenderContext const& get_context() const;
    std::unique_ptr<GBuffer> const& get_gbuffer() const;
    LightTable& get_light_table();

    void bind_gbuffer_input(std::shared_ptr<ShaderProgram> const& shader) const;
    void bind_light_table(std::shared_ptr<ShaderProgram> const& shader) const;
    void draw_quad();
    void draw_quad_instanced();

    void draw_box();



#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
    // time queries
    void begin_gpu_query(RenderContext const& ctx, std::string const& query_name);
    void end_gpu_query(RenderContext const& ctx, std::string const& query_name);

    void begin_cpu_query(std::string const& query_name);
    void end_cpu_query(std::string const& query_name);

    void fetch_gpu_query_results(RenderContext const& ctx);
#endif

    void clear_frame_cache();

  private:
    void bind_camera_uniform_block(unsigned location) const;
    void bind_light_transformation_uniform_block(unsigned location) const;

    void render_shadow_map(LightTable::LightBlock& light_block, Frustum const& frustum, unsigned cascade_id, unsigned viewport_size, bool redraw);

    void generate_shadow_map_sunlight(
        std::shared_ptr<ShadowMap> const& shadowmap, node::LightNode& light, LightTable::LightBlock& light_block, unsigned viewport_size, bool redraw, math::mat4 const& original_screen_transform);

    void generate_shadow_map_pointlight(std::shared_ptr<ShadowMap> const& shadowmap, node::LightNode& light, LightTable::LightBlock& light_block, unsigned viewport_size, bool redraw);

    void generate_shadow_map_spotlight(node::LightNode& light, LightTable::LightBlock& light_block, unsigned viewport_size, bool redraw);

    PipelineViewState current_viewstate_;

    RenderContext& context_;
    std::unique_ptr<GBuffer> gbuffer_;
    std::shared_ptr<SharedShadowMapResource> shadow_map_res_;
    CameraUniformBlock camera_block_;
    std::unique_ptr<LightTable> light_table_;
    LightTransformationUniformBlock light_transform_block_;

    math::vec2ui last_resolution_;
    PipelineDescription last_description_;
    SubstitutionMap global_substitution_map_;

    std::vector<PipelinePass> passes_;
    std::vector<PipelineResponsibility> responsibilities_pre_render_;
    std::vector<PipelineResponsibility> responsibilities_post_render_;
    scm::gl::quad_geometry_ptr quad_;
    scm::gl::box_geometry_ptr box_;
#ifdef GUACAMOLE_ENABLE_PIPELINE_PASS_TIME_QUERIES
#define GUA_ENABLE_PROFILING_TIME_QUERIES
    time_query_collection queries_;
#endif
};

} // namespace gua

#endif // GUA_PIPELINE_HPP
