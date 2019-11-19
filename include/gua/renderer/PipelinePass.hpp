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

#ifndef GUA_PIPELINE_PASS_HPP
#define GUA_PIPELINE_PASS_HPP

#include <scm/gl_core.h>
#include <gua/renderer/ShaderProgram.hpp>

namespace gua
{
class Pipeline;
class PipelinePass;
class PipelinePassDescription;
class PipelineResponsibility;
class PipelineResponsibilityDescription;
struct RenderContext;

enum class RenderMode
{
    Custom,
    Callback,
    Quad
};

enum class OcclusionCullingMode {
    No_Culling,
    Naive_Stop_And_Wait,
    Hierarchical_Stop_And_Wait,
    Coherent_Hierarchical_Culling,
    
    Num_Occlusion_Culling_Modes
};


class GUA_DLL PipelinePassPrivate
{
  public:
    boost::optional<scm::gl::rasterizer_state_desc> rasterizer_state_desc_;
    boost::optional<scm::gl::depth_stencil_state_desc> depth_stencil_state_desc_;
    boost::optional<scm::gl::blend_state_desc> blend_state_desc_;

    bool needs_color_buffer_as_input_{false};
    bool writes_only_color_buffer_{false};
    bool enable_for_shadows_{false};
    bool is_enabled_{true}; //for toggling passes on or off

    RenderMode rendermode_{RenderMode::Custom};
    std::string name_{"PipelinePass"};

    std::function<void(PipelinePass&, PipelinePassDescription const&, Pipeline&)> process_;
};

class GUA_DLL PipelinePassDescription
{
  public:
    friend class Pipeline;
    friend class PipelinePass;

    virtual std::shared_ptr<PipelinePassDescription> make_copy() const = 0;
    virtual ~PipelinePassDescription() {}

    void touch();
    std::string const& name() const;
    unsigned mod_count() const;

  protected:    
    virtual PipelinePass make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) = 0;
    friend bool operator!=(PipelinePassDescription const& lhs, PipelinePassDescription const& rhs) { return lhs.mod_count_ != rhs.mod_count_; };

    // shader names
    std::string vertex_shader_ = "";
    std::string fragment_shader_ = "";
    std::string geometry_shader_ = "";

    bool vertex_shader_is_file_name_ = true;
    bool fragment_shader_is_file_name_ = true;
    bool geometry_shader_is_file_name_ = true;

    unsigned mod_count_ = 0;
    mutable bool recompile_shaders_ = true;

    PipelinePassPrivate private_;
    std::vector<std::shared_ptr<PipelineResponsibilityDescription>> pipeline_responsibilities_;

  public:
    std::map<std::string, UniformValue> uniforms;

    void set_user_data(void* data) { user_data_ = data; }
    void* get_user_data() const { return user_data_; }

    const std::vector<std::shared_ptr<PipelineResponsibilityDescription>>& get_responsibilities() const;

    // getter and setter for occlusion culling render modes
    OcclusionCullingMode get_occlusion_culling_mode() const;
    void set_occlusion_culling_mode(OcclusionCullingMode const& oc_mode);

    void enable(bool enable);
    bool is_enabled() const;
  private:
    void* user_data_ = nullptr;


    // global occlusion mode for the entire pipeline. can be queried in the renderer
    OcclusionCullingMode occlusion_culling_mode_ = OcclusionCullingMode::No_Culling;
};

class GUA_DLL PipelinePass
{
  public:
    friend class Pipeline;

    bool needs_color_buffer_as_input() const;
    bool writes_only_color_buffer() const;
    bool enable_for_shadows() const;

    scm::gl::rasterizer_state_ptr rasterizer_state() const;
    scm::gl::depth_stencil_state_ptr depth_stencil_state() const;
    scm::gl::blend_state_ptr blend_state() const;
    std::shared_ptr<ShaderProgram> shader() const;

    void process(PipelinePassDescription const& desc, Pipeline& pipe);

    PipelinePass(PipelinePassDescription const&, RenderContext const&, SubstitutionMap const&);
    ~PipelinePass() = default;

    virtual void on_delete(Pipeline* pipe) {}

  protected:
    PipelinePass() = default;

    virtual void upload_program(PipelinePassDescription const& desc, RenderContext const& ctx);

    scm::gl::rasterizer_state_ptr rasterizer_state_;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_;
    scm::gl::blend_state_ptr blend_state_;
    std::shared_ptr<ShaderProgram> shader_;

  private:
    PipelinePassPrivate private_;
    SubstitutionMap substitution_map_;
};

class GUA_DLL PipelineResponsibilityPrivate
{
  public:
    enum TYPE
    {
        PRE_RENDER = 0,
        POST_RENDER = 1
    };

    TYPE type_{PRE_RENDER};
    std::string name_{""};
    std::function<void(Pipeline& pipe)> fulfil_ = [](Pipeline& pipe) {};
};

class GUA_DLL PipelineResponsibility
{
  public:
    friend class PipelineResponsibilityDescription;

    ~PipelineResponsibility() = default;

    PipelineResponsibilityPrivate::TYPE type() const { return private_.type_; }
    std::string name() const { return private_.name_; }

    void fulfil(Pipeline& pipe) { private_.fulfil_(pipe); }

  protected:
    PipelineResponsibility(PipelineResponsibilityDescription const& desc, Pipeline& pipe);

  private:
    PipelineResponsibilityPrivate private_;
};

class GUA_DLL PipelineResponsibilityDescription
{
  public:
    friend class Pipeline;
    friend class PipelineResponsibility;

    PipelineResponsibilityDescription() = default;
    ~PipelineResponsibilityDescription() = default;

  protected:
    virtual PipelineResponsibility make_responsibility(Pipeline& pipe)
    {
        PipelineResponsibility responsibility(*this, pipe);
        return responsibility;
    };

    PipelineResponsibilityPrivate private_;
    friend bool operator==(PipelineResponsibilityDescription const& lhs, PipelineResponsibilityDescription const& rhs);
    friend bool operator!=(PipelineResponsibilityDescription const& lhs, PipelineResponsibilityDescription const& rhs);
};

} // namespace gua

#endif // GUA_PIPELINE_PASS_HPP
