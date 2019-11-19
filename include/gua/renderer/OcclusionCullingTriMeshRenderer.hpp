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

#ifndef GUA_OCCLUSION_CULLING_TRIMESH_RENDERER_HPP
#define GUA_OCCLUSION_CULLING_TRIMESH_RENDERER_HPP

#include <map>
#include <unordered_map>

#include <gua/platform.hpp>
#include <gua/config.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <scm/gl_core/shader_objects.h>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
#include <gua/renderer/VTRenderer.hpp>
#endif

namespace gua
{
class MaterialShader;
class Pipeline;
class PipelinePassDescription;
class RenderTarget;

enum class OcclusionCullingMode;

class GUA_DLL OcclusionCullingTriMeshRenderer
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    : public VTRenderer
#endif
{
  public:
    OcclusionCullingTriMeshRenderer(RenderContext const& ctx, SubstitutionMap const& smap);

    /* main render call which internally calls the different occlusion culling supported render functions
       based on the set render mode 
    */
    void render(Pipeline& pipe, PipelinePassDescription const& desc);

    // occlusion culling supported render functions
    void render_without_oc(Pipeline& pipe, PipelinePassDescription const& desc);
    void render_naive_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc);
    void render_hierarchical_stop_and_wait_oc(Pipeline& pipe, PipelinePassDescription const& desc);

    void switch_state_for_depth_complexity_vis(RenderContext const& ctx, std::shared_ptr<ShaderProgram>& active_shader);
    void switch_state_based_on_node_material(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader, 
                                             MaterialShader* current_material, RenderTarget const& target, bool shadow_mode, std::size_t cam_uuid);

    void upload_uniforms_for_node(RenderContext const& ctx, node::TriMeshNode* tri_mesh_node, std::shared_ptr<ShaderProgram>& current_shader, 
                                  Pipeline& pipe, scm::gl::rasterizer_state_ptr& current_rasterizer_state);

  private:

    // different rasterizer states for different render modes
    scm::gl::rasterizer_state_ptr rs_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_cull_none_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_none_ = nullptr;

    // different depth stencil states for different effects
        // default state enables depth testing and depth writing
    scm::gl::depth_stencil_state_ptr default_depth_test_ = nullptr;
        // this depth stencil state disables depth testing and depth writing for the depth complexity visualization
    scm::gl::depth_stencil_state_ptr depth_stencil_state_no_test_no_writing_state_ = nullptr;

    //this depth stencil state is supposed to be used for issueing occlusion queries
    scm::gl::depth_stencil_state_ptr depth_stencil_state_test_without_writing_state_ = nullptr;
    
    // blend states telling opengl what to do with new fragments
    // default state just writes the attributes of the latest accepted fragment over the previous one
    scm::gl::blend_state_ptr default_blend_state_ = nullptr;
    // this accumulation state adds the color of all fragments on top of each other.
    // we use this in combination with disabled depth tests to do the depth complexity visualization
    scm::gl::blend_state_ptr color_accumulation_state_ = nullptr;

    // this accumulation state adds the color of all fragments on top of each other.
    // we use this in combination with disabled depth tests to do the depth complexity visualization
    scm::gl::blend_state_ptr color_masks_disabled_state_ = nullptr;

    // these shaders are used when we decide to actually draw geometry
    // there map contains one shader program for any material that we encounter
    std::vector<ShaderProgramStage> default_rendering_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> default_rendering_programs_;


    // these shaders and the compilshaders are used in combination with hardware occlusion queries
    std::vector<ShaderProgramStage> occlusion_query_program_stages_;
    std::shared_ptr<ShaderProgram> occlusion_query_programs_;


    // these shaders are used only for visualizing the depth complexity in our system, together with disabled
    // depth tests and color accumulation blend states (expensive, but nevertheless only used for debug purposes)
    std::vector<ShaderProgramStage> depth_complexity_vis_program_stages_;
    std::shared_ptr<ShaderProgram> depth_complexity_vis_program_;



    SubstitutionMap global_substitution_map_;

};

} // namespace gua

#endif // GUA_OCCLUSION_CULLING_TRIMESH_RENDERER_HPP
