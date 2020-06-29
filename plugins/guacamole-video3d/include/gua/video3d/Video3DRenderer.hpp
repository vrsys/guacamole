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

#ifndef GUA_VIDEO3D_RENDERER_HPP
#define GUA_VIDEO3D_RENDERER_HPP

#include <array>

// guacamole headers
#include <gua/video3d/platform.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <unordered_map>

namespace video3d
{
class NetKinectArray;
}

namespace gua
{
class ShaderProgram;
class Pipeline;
class MaterialShader;
class Video3DResource;

class GUA_VIDEO3D_DLL Video3DRenderer
{
  public:
    enum pass
    {
        warp_pass = 0,
        blend_pass = 1
    };

  public:
    Video3DRenderer();

    void render(Pipeline& pipe, PipelinePassDescription const& desc);

    // /*virtual*/ void draw   (RenderContext const& context,
    //                          std::string const& ksfile_name,
    //                          std::string const& material_name,
    //                          scm::math::mat4 const& model_matrix,
    //                          scm::math::mat4 const& normal_matrix,
    //                          Frustum const& frustum,
    //                          View const& view) const;

    void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

    void draw_video3dResource(RenderContext& ctx, Video3DResource const& video3d);
    void update_buffers(RenderContext const& ctx, Video3DResource const& video3d, Pipeline& pipe);

  private: // attributes
    void process_textures(RenderContext const& ctx, Video3DResource const& res, Pipeline& pipe);

    bool initialized_;

    std::vector<ShaderProgramStage> program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> programs_;
    SubstitutionMap global_substitution_map_;

    std::shared_ptr<ShaderProgram> warp_pass_program_;

    static const unsigned MAX_NUM_KINECTS = 6;

    scm::gl::texture_2d_ptr warp_depth_result_;
    scm::gl::texture_2d_ptr warp_color_result_;
    scm::gl::frame_buffer_ptr warp_result_fbo_;

    scm::gl::rasterizer_state_ptr no_bfc_rasterizer_state_;
    scm::gl::sampler_state_ptr nearest_sampler_state_;
    scm::gl::sampler_state_ptr linear_sampler_state_;

    scm::gl::depth_stencil_state_ptr depth_stencil_state_warp_pass_;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_blend_pass_;

    struct Video3DData
    {
        Video3DData() = default;
        Video3DData(RenderContext const& ctx, Video3DResource const& video3d);
        // gl resources
        scm::gl::rasterizer_state_ptr rstate_solid_ = nullptr;
        scm::gl::texture_2d_ptr color_tex_ = nullptr;
        scm::gl::texture_2d_ptr depth_tex_ = nullptr;
        scm::gl::texture_2d_ptr depth_tex_processed_ = nullptr;

        // cpu resources
        std::shared_ptr<video3d::NetKinectArray> nka_ = nullptr;
        std::vector<scm::gl::texture_3d_ptr> cv_xyz_ = {};
        std::vector<scm::gl::texture_3d_ptr> cv_uv_ = {};
        unsigned frame_counter_ = 0;
    };

    std::unordered_map<std::size_t, Video3DData> video3Ddata_;

    std::unordered_map<std::size_t, scm::gl::buffer_ptr> kinect_textures_per_context_pbo_back_;// = std::vector<std::vector<scm::gl::buffer_ptr>>(MAX_NUM_SUPPORTED_CONTEXTS, std::vector<scm::gl::buffer_ptr>(4, nullptr));
    std::unordered_map<std::size_t, scm::gl::buffer_ptr> kinect_textures_per_context_pbo_;// = std::vector<std::vector<scm::gl::buffer_ptr>>(MAX_NUM_SUPPORTED_CONTEXTS, std::vector<scm::gl::buffer_ptr>(4, nullptr));
    std::unordered_map<std::size_t, uint8_t*> mapped_pbo_pointers_;


    scm::gl::frame_buffer_ptr fbo_depth_process_;
    std::shared_ptr<ShaderProgram> depth_process_program_;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_tex_process_;


};

} // namespace gua

#endif // GUA_VIDEO3D_RENDERER_HPP
