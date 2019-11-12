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

class GUA_DLL OcclusionCullingTriMeshRenderer
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    : public VTRenderer
#endif
{
  public:
    OcclusionCullingTriMeshRenderer(RenderContext const& ctx, SubstitutionMap const& smap);

    void render(Pipeline& pipe, PipelinePassDescription const& desc);

  private:
    scm::gl::rasterizer_state_ptr rs_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_cull_none_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_back_ = nullptr;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_none_ = nullptr;

    scm::gl::depth_stencil_state_ptr default_depth_test_ = nullptr;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_no_test_no_writing_state_ = nullptr;

    scm::gl::blend_state_ptr default_blend_state_ = nullptr;   
    scm::gl::blend_state_ptr color_accumulation_state_ = nullptr;

    std::vector<ShaderProgramStage> depth_complexity_vis_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> depth_complexity_vis_programs_;

    std::vector<ShaderProgramStage> standard_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> standard_programs_;

    SubstitutionMap global_substitution_map_;
};

} // namespace gua

#endif // GUA_OCCLUSION_CULLING_TRIMESH_RENDERER_HPP
