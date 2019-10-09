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

#ifndef GUA_DYNAMIC_LINE_RENDERER_HPP
#define GUA_DYNAMIC_LINE_RENDERER_HPP

#include <map>
#include <unordered_map>

#include <gua/platform.hpp>
#include <gua/renderer/ShaderProgram.hpp>

#include <scm/gl_core/shader_objects.h>

namespace gua
{
class MaterialShader;
class Pipeline;
class PipelinePassDescription;

class DynamicLineRenderer
{
  public:
    DynamicLineRenderer(RenderContext const& ctx, SubstitutionMap const& smap);

    void render(Pipeline& pipe, PipelinePassDescription const& desc);

  private:
    scm::gl::rasterizer_state_ptr rs_cull_back_;
    scm::gl::rasterizer_state_ptr rs_cull_none_;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_back_;
    scm::gl::rasterizer_state_ptr rs_wireframe_cull_none_;

    std::vector<ShaderProgramStage> program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> programs_;

    std::vector<ShaderProgramStage> volumetric_point_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> volumetric_point_programs_;

    std::vector<ShaderProgramStage> volumetric_line_program_stages_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> volumetric_line_programs_;

    SubstitutionMap global_substitution_map_;
};

} // namespace gua

#endif // GUA_DYNAMIC_LINE_RENDERER_HPP
