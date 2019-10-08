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

#ifndef GUA_SKELETAL_ANIMATION_RENDERER_HPP
#define GUA_SKELETAL_ANIMATION_RENDERER_HPP

// guacamole headers
#include <gua/skelanim/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/skelanim/renderer/BoneTransformUniformBlock.hpp>

// external headers
#include <scm/gl_core/shader_objects.h>
#include <unordered_map>

namespace gua
{
class MaterialShader;
class Pipeline;
class PipelinePassDescription;

/**
 * @brief holds bone mapping offsets
 * @details holds info about where to read from the bonetransformblock buffers
 */
struct SharedSkinningResource
{
    scm::gl::buffer_ptr bone_ids_ = nullptr;
    scm::gl::buffer_ptr bone_weights_ = nullptr;
    size_t offset_bytes = 0;
};
class GUA_SKELANIM_DLL SkeletalAnimationRenderer
{
  public:
    SkeletalAnimationRenderer(RenderContext const& ctx);
    virtual ~SkeletalAnimationRenderer() {}

    void render(Pipeline& pipe, PipelinePassDescription const& desc);

    void set_global_substitution_map(SubstitutionMap const& smap) { global_substitution_map_ = smap; }

    void create_state_objects(RenderContext const& ctx);

  private:
    scm::gl::rasterizer_state_ptr rs_cull_back_;
    scm::gl::rasterizer_state_ptr rs_cull_none_;

    std::vector<ShaderProgramStage> program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> programs_;
    SubstitutionMap global_substitution_map_;

    BoneTransformUniformBlock bones_block_;

    SharedSkinningResource skinning_resource_;

    unsigned last_frame_;
};

} // namespace gua

#endif // GUA_SKELETAL_ANIMATION_RENDERER_HPP
