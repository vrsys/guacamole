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

#ifndef GUA_P_LOD_RENDERER_HPP
#define GUA_P_LOD_RENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

// guacamole headers
#include <gua/renderer/PLodPass.hpp>

#include <gua/renderer/PLodSharedResources.hpp>

#include <gua/renderer/PLodSubRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ResourceFactory.hpp>
#include <gua/renderer/OcclusionCullingAwareRenderer.hpp>

#include <gua/node/PLodNode.hpp>

// external headers
#include <lamure/ren/cut_database_record.h>

namespace gua
{
class MaterialShader;
class ShaderProgram;
// class plod_shared_resources;

class GUA_LOD_DLL PLodRenderer : public OcclusionCullingAwareRenderer
{
  public:
    PLodRenderer();

    void render(Pipeline& pipe, PipelinePassDescription const& desc);
    void renderSingleNode(Pipeline& pipe, PipelinePassDescription const& desc, gua::node::Node* const current_node, RenderInfo& current_render_info) override;
    void set_global_substitution_map(SubstitutionMap const& smap);

  private: // shader related auxiliary methods
    void perform_frustum_culling_for_scene(std::vector<node::Node*>& models,
                                           std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& culling_results_per_model,
                                           std::unordered_map<node::PLodNode*, lamure::ren::cut*> cut_map,
                                           lamure::ren::camera const& cut_update_cam,
                                           gua::Pipeline& pipe) const;

    void _create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims);

    void _check_for_resource_updates(gua::Pipeline const& pipe, RenderContext const& ctx);

  private: // out-of-core related auxiliary methods
    lamure::context_t _register_context_in_cut_update(gua::RenderContext const& ctx);

  private: // misc auxiliary methods
    bool _intersects(scm::gl::boxf const& bbox, std::vector<math::vec4> const& global_planes) const;

    std::vector<math::vec3> _get_frustum_corners_vs(gua::Frustum const& frustum) const;

  private: // member variables
    std::map<PLodPassDescription::SurfelRenderMode, std::shared_ptr<std::vector<std::shared_ptr<PLodSubRenderer>>>> plod_pipelines_;
    std::vector<std::map<lamure::model_t, std::vector<bool>>> model_frustum_culling_results_;
    std::map<std::size_t, std::pair<gua::math::vec2ui, gua::plod_shared_resources>> shared_pass_resources_;

    unsigned previous_frame_count_;

    // CPU resources
    SubstitutionMap global_substitution_map_;
    ResourceFactory factory_;
};

} // namespace gua

#endif // GUA_P_LOD_RENDERER_HPP
