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

#ifndef GUA_M_LOD_RENDERER_HPP
#define GUA_M_LOD_RENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

#include <gua/platform.hpp>
#include <gua/config.hpp>

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ResourceFactory.hpp>

// external headers
#include <lamure/ren/cut_database_record.h>

#include <gua/node/MLodNode.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
#include <gua/renderer/VTRenderer.hpp>
#endif

namespace gua
{
class MaterialShader;
class Pipeline;
class PipelinePassDescription;
class ShaderProgram;

class GUA_LOD_DLL MLodRenderer
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    : public VTRenderer
#endif
{
  public:
    MLodRenderer(RenderContext const& ctx, SubstitutionMap const& smap);

    void render(Pipeline& pipe, PipelinePassDescription const& desc);
    void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

    void create_state_objects(RenderContext const& ctx);

  private: // lamure auxiliary methods
    std::shared_ptr<ShaderProgram> _get_material_program(MaterialShader* material, std::shared_ptr<ShaderProgram> const& current_program, bool& program_changed, gua::node::MLodNode* mlod_node);

    void _initialize_tri_mesh_lod_program(MaterialShader* material, gua::node::MLodNode* mlod_node);

    lamure::context_t _lamure_register_context(gua::RenderContext const& ctx);

    bool _intersects(scm::gl::boxf const& bbox, std::vector<math::vec4> const& global_planes) const;

    std::vector<math::vec3> _get_frustum_corners_vs(gua::Frustum const& frustum) const;

  private: // member variables
    bool gpu_resources_created_;
    unsigned previous_frame_count_;

    std::mutex mutex_;

    scm::gl::rasterizer_state_ptr rs_cull_back_;
    scm::gl::rasterizer_state_ptr rs_cull_none_;

    unsigned current_rendertarget_width_;
    unsigned current_rendertarget_height_;

    std::vector<ShaderProgramStage> program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> programs_;
    SubstitutionMap global_substitution_map_;
};

} // namespace gua

#endif // GUA_M_LOD_RENDERER_HPP
