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

#ifndef GUA_PLOD_SUB_RENDERER_HPP
#define GUA_PLOD_SUB_RENDERER_HPP

#include <gua/renderer/Pipeline.hpp>

#include <gua/renderer/PLodSharedResources.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/node/PLodNode.hpp>
//#include <gua/renderer/PLodRenderer.hpp>

#include <lamure/ren/camera.h>
#include <lamure/ren/policy.h>
#include <lamure/ren/dataset.h>
#include <lamure/ren/model_database.h>
#include <lamure/ren/cut_database.h>
#include <lamure/ren/controller.h>

namespace gua
{
class GUA_LOD_DLL PLodSubRenderer
{
  public:
    PLodSubRenderer();

    virtual void render_sub_pass(Pipeline& pipe,
                                 PipelinePassDescription const& desc,
                                 gua::plod_shared_resources& shared_resources,
                                 std::vector<node::Node*>& sorted_models,
                                 std::unordered_map<node::PLodNode*, std::unordered_set<lamure::node_t>>& nodes_in_frustum_per_model,
                                 lamure::context_t context_id,
                                 lamure::view_t lamure_view_id) = 0;

    virtual void forward_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

    virtual void create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims, gua::plod_shared_resources& shared_resources){};

  protected: // shader related auxiliary methods
    virtual void _initialize_shader_program();
    virtual void _initialize_material_dependent_shader_programs(MaterialShader* material);

    virtual std::shared_ptr<ShaderProgram> _get_material_dependent_shader_program(MaterialShader* material, std::shared_ptr<ShaderProgram> const& current_program, bool& program_changed);

    // virtual void _upload_model_dependent_uniforms(RenderContext const& ctx, node::PLodNode* plod_node, gua::Pipeline& pipe) {};

    virtual void _check_for_shader_program();

    virtual void _register_shared_resources(gua::plod_shared_resources& shared_resources);

    std::vector<ShaderProgramStage> shader_stages_;
    std::shared_ptr<ShaderProgram> shader_program_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> material_dependent_shader_programs_;

    // std::map<plod_shared_resources::AttachmentID, scm::gl::texture_2d_ptr> texture_resource_ptrs_;
    gua::plod_shared_resources resource_ptrs_;

    // std::map<plod_shared_resources::

    scm::gl::frame_buffer_ptr custom_FBO_ptr_;

    SubstitutionMap global_substitution_map_;

    bool shaders_loaded_;
    bool gpu_resources_already_created_;
    unsigned current_rendertarget_width_;
    unsigned current_rendertarget_height_;
};
} // namespace gua

#endif // GUA_PLOD_SUB_RENDERER_HPP