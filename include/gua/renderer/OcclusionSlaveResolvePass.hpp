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

#ifndef GUA_OCCLUSION_SLAVE_RESOLVE_PASS_HPP
#define GUA_OCCLUSION_SLAVE_RESOLVE_PASS_HPP

#include <gua/renderer/PipelinePass.hpp>

#include <memory>

namespace gua
{
class Pipeline;

class GUA_DLL OcclusionSlaveResolvePassDescription : public PipelinePassDescription
{
  public:
    OcclusionSlaveResolvePassDescription();

    std::shared_ptr<PipelinePassDescription> make_copy() const override;
    friend class Pipeline;

  protected:
    void create_gpu_resources(gua::RenderContext const& ctx, scm::math::vec2ui const& render_target_dims);

    PipelinePass make_pass(RenderContext const&, SubstitutionMap&) override;
    mutable int last_rendered_view_id;
    mutable int last_rendered_side;

    scm::math::vec2ui gbuffer_extraction_resolution_;

    std::vector<ShaderProgramStage> control_monitor_shader_stages_;
    std::shared_ptr<ShaderProgram> control_monitor_shader_program_;

    std::vector<ShaderProgramStage> depth_downsampling_shader_stages_;
    std::shared_ptr<ShaderProgram> depth_downsampling_shader_program_;

    bool gpu_resources_already_created_;

    scm::gl::depth_stencil_state_ptr no_depth_test_depth_stencil_state_;
    scm::gl::depth_stencil_state_ptr always_write_depth_stencil_state_;

    scm::gl::sampler_state_ptr nearest_sampler_state_;

    scm::gl::frame_buffer_ptr depth_buffer_downsampling_fbo_;
    scm::gl::texture_2d_ptr downsampled_depth_attachment_;
};
} // namespace gua

#endif // GUA_OCCLUSION_SLAVE_RESOLVE_PASS_HPP
