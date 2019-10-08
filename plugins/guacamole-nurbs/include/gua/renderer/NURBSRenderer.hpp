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
#ifndef GUA_NURBS_RENDERER_HPP
#define GUA_NURBS_RENDERER_HPP

#include <string>
#include <map>
#include <unordered_map>

// guacamole headers
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/ResourceFactory.hpp>

namespace gua
{
class MaterialShader;
class ShaderProgram;

/////////////////////////////////////////////////////////////////////////////////////////////
// Transformation Feedback Specific Members - only once per context
/////////////////////////////////////////////////////////////////////////////////////////////
struct GUA_NURBS_DLL NURBSTransformFeedbackBuffer : public PluginRessource
{
    static const unsigned GUA_TRANSFORM_FEEDBACK_BUFFER_BASE_CACHE_ID = 1;

    scm::gl::transform_feedback_ptr _transform_feedback;
    scm::gl::vertex_array_ptr _transform_feedback_vao;
    scm::gl::buffer_ptr _transform_feedback_vbo;
};

struct GUA_NURBS_DLL NURBSRasterizationState : public PluginRessource
{
    static const unsigned GUA_RASTERIZATION_BASE_CACHE_ID = 2;

    scm::gl::sampler_state_ptr _sstate;
    scm::gl::rasterizer_state_ptr _wire_no_cull;
    scm::gl::rasterizer_state_ptr _solid_cull;
    scm::gl::rasterizer_state_ptr _solid_no_cull;
};

class GUA_NURBS_DLL NURBSRenderer
{
  public:
    static const unsigned GUA_MAX_XFB_BUFFER_SIZE_IN_BYTES = 1024000000; // reserve GB transform feedback buffer
    static const unsigned GUA_HULLVERTEXMAP_SSBO_BINDING = 1;
    static const unsigned GUA_ATTRIBUTE_SSBO_BINDING = 2;
    static const unsigned GUA_ATOMIC_COUNTER_BINDING = 3;
    static const unsigned GUA_FEEDBACK_BUFFER_BINDING = 4;
    // static const unsigned GUA_ABUFFER_MAX_FRAGMENTS = 10000000; // 10M fragments
    static const unsigned GUA_ANTI_ALIASING_MODE = 0;
    static const unsigned GUA_WRITE_DEBUG_COUNTER = 0;
    static const unsigned GUA_SECOND_PASS_TRIANGLE_TESSELATION = 0;
    static const unsigned GUA_MAX_FEEDBACK_BUFFER_INDICES = 0;

  public:
    NURBSRenderer();
    ~NURBSRenderer();

    void render(Pipeline& pipe, PipelinePassDescription const& desc);
    void set_substitutions(SubstitutionMap const& smap);

    void pretessellation(bool enable);
    bool pretessellation() const;

  private: // auxiliary methods
    void _load_shaders();
    void _initialize_pre_tesselation_program(RenderContext const& ctx);
    void _initialize_tesselation_program(MaterialShader*);

    std::shared_ptr<ShaderProgram> _get_material_program(MaterialShader* material, std::shared_ptr<ShaderProgram> const& current_program, bool& program_changed);

    void _reset();

    std::string _transform_feedback_vertex_shader() const;
    std::string _transform_feedback_geometry_shader() const;
    std::string _transform_feedback_tess_control_shader() const;
    std::string _transform_feedback_tess_evaluation_shader() const;

    std::string _final_vertex_shader() const;
    std::string _final_tess_control_shader() const;
    std::string _final_tess_evaluation_shader() const;
    std::string _final_geometry_shader() const;
    std::string _final_fragment_shader() const;

  private: // attributes
    unsigned current_modcount_;
    SubstitutionMap global_substitution_map_;
    ResourceFactory factory_;

    // CPU Ressources
    std::vector<ShaderProgramStage> pre_tesselation_shader_stages_;
    std::list<std::string> pre_tesselation_interleaved_stream_capture_;
    std::vector<ShaderProgramStage> tesselation_shader_stages_;

    // GPU Ressources
    std::shared_ptr<ShaderProgram> pre_tesselation_program_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> tesselation_programs_;

    bool _enable_triangular_tesselation = false;
    bool _enable_count = false;
    bool _antialiasing = false;
    bool _enable_holefilling = false;
    bool _pretessellation = true;
};

} // namespace gua

#endif // GUA_NURBS_RENDERER_HPP
