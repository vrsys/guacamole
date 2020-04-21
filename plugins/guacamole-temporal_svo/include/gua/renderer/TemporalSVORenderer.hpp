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

#ifndef GUA_TEMPORAL_SVO_RENDERER_HPP
#define GUA_TEMPORAL_SVO_RENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

// guacamole headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/View.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/ResourceFactory.hpp>

#include <gua/node/TemporalSVONode.hpp>
//#include <gua/renderer/TemporalSVOResource.hpp>

// external headers

namespace gua
{
// forward declarations
class MaterialShader;
class ShaderProgram;

using MaterialProgramsMap = std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>>;

class TemporalSVORenderer
{
  public:
    TemporalSVORenderer(gua::RenderContext const& ctx, gua::SubstitutionMap const& substitution_map);

    void render(Pipeline& pipe, PipelinePassDescription const& desc);

    void reload_programs();

  protected: // shader related auxiliary methods
    virtual void _load_shaders();
    void _initialize_volume_raycasting_programs();
    void _initialize_volume_compositing_programs();

    std::shared_ptr<ShaderProgram> _get_material_program(MaterialShader* material,
                                                         std::shared_ptr<ShaderProgram> const& current_program,
                                                         bool& program_changed);

    //void _initialize_ray_casting_program(MaterialShader* material, CompressionMode const c_mode, SpatialFilterMode const sf_mode, TemporalFilterMode const tf_mode, NodeRenderMode const r_mode);

    void _create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims);

    void _check_for_resource_updates(gua::Pipeline const& pipe, RenderContext const& ctx);

  private: // misc auxiliary methods
           /*
            bool _intersects(scm::gl::boxf const& bbox,
                             std::vector<math::vec4f> const& global_planes) const;
           */

  protected: // member variables
    // FBOs:
    //////////////////////////////////////////////////////////////////////////////////////
    scm::gl::frame_buffer_ptr volume_raycasting_fbo_;

    // accumulation pass FBO & attachments
    // scm::gl::texture_2d_ptr                      volume_raycasting_color_result_;
    // scm::gl::texture_2d_ptr                      volume_raycasting_depth_result_;

    // schism-GL states:
    //////////////////////////////////////////////////////////////////////////////////////
    scm::gl::rasterizer_state_ptr no_backface_culling_rasterizer_state_;
    scm::gl::rasterizer_state_ptr frontface_culling_rasterizer_state_;

    scm::gl::sampler_state_ptr nearest_sampler_state_;
    scm::gl::sampler_state_ptr trilin_sampler_state_;

    // misc:
    ////////////////////////////////////////////////////////////////////////////////////
    scm::gl::quad_geometry_ptr fullscreen_quad_;

    scm::gl::vertex_array_ptr box_vertex_array_;
    scm::gl::buffer_ptr box_vertex_buffer_;
    scm::gl::buffer_ptr box_element_buffer_;

    bool gpu_resources_already_created_;
    unsigned previous_frame_count_;

    // context guard
    ////////////////////////////////////////////////////////////////////////////////////
    bool shaders_loaded_;

    math::vec2ui current_rendertarget_dims_;
    // additional GPU resources
    std::vector<ShaderProgramStage> forward_cube_shader_stages_;

    std::shared_ptr<ShaderProgram> forward_cube_shader_program_;

    std::vector<ShaderProgramStage> compositing_shader_stages_;
    std::shared_ptr<ShaderProgram> compositing_shader_program_;

    SubstitutionMap global_substitution_maps_;
    ResourceFactory factory_;

    std::vector<ShaderProgramStage> ray_casting_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> ray_casting_programs_uncompressed_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> ray_casting_programs_compressed_;

};

} // namespace gua

#endif // GUA_TEMPORAL_SVO_RENDERER_HPP
