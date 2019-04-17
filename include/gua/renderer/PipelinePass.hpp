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
struct RenderContext;

enum class RenderMode
{
    Custom,
    Callback,
    Quad
};

class GUA_DLL PipelinePassDescription
{
  public:
    virtual std::shared_ptr<PipelinePassDescription> make_copy() const = 0;
    virtual ~PipelinePassDescription() {}

    friend class Pipeline;
    friend class PipelinePass;

    void touch();
    std::string const& name() const;
    unsigned mod_count() const;

  protected:
    virtual PipelinePass make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map) = 0;
    friend bool operator!=(PipelinePassDescription const& lhs, PipelinePassDescription const& rhs) { return lhs.mod_count_ != rhs.mod_count_; };

    virtual void apply_post_render_action(RenderContext const& ctx, gua::Pipeline* pipe) const {};
    // shader names
    std::string vertex_shader_ = "";
    std::string fragment_shader_ = "";
    std::string geometry_shader_ = "";
    std::string name_ = "";

    bool vertex_shader_is_file_name_ = true;
    bool fragment_shader_is_file_name_ = true;
    bool geometry_shader_is_file_name_ = true;

    bool needs_color_buffer_as_input_ = false;
    bool writes_only_color_buffer_ = false;

    bool enable_for_shadows_ = false;
    unsigned mod_count_ = 0;

    mutable bool recompile_shaders_ = true;

    RenderMode rendermode_ = RenderMode::Custom;

    boost::optional<scm::gl::rasterizer_state_desc> rasterizer_state_;
    boost::optional<scm::gl::blend_state_desc> blend_state_;
    boost::optional<scm::gl::depth_stencil_state_desc> depth_stencil_state_;

    std::function<void(PipelinePass&, PipelinePassDescription const&, Pipeline&)> process_ = [](PipelinePass&, PipelinePassDescription const&, Pipeline&) {};

  public:
    std::map<std::string, UniformValue> uniforms;

    void set_user_data(void* data) { user_data_ = data; }

    void* get_user_data() const { return user_data_; }

  private:
    void* user_data_ = nullptr;
};

class GUA_DLL PipelinePass
{
  public:
    inline bool needs_color_buffer_as_input() const { return needs_color_buffer_as_input_; }
    inline bool writes_only_color_buffer() const { return writes_only_color_buffer_; }
    inline bool enable_for_shadows() const { return enable_for_shadows_; }

    void process(PipelinePassDescription const& desc, Pipeline& pipe);

    virtual void on_delete(Pipeline* pipe) {}

    friend class Pipeline;

  protected:
  public: // for refactoring purposes
    PipelinePass() = default;
    PipelinePass(PipelinePassDescription const&, RenderContext const&, SubstitutionMap const&);

    virtual void upload_program(PipelinePassDescription const& desc, RenderContext const& ctx);

    std::shared_ptr<ShaderProgram> shader_ = nullptr;

    scm::gl::rasterizer_state_ptr rasterizer_state_ = nullptr;
    scm::gl::depth_stencil_state_ptr depth_stencil_state_ = nullptr;
    scm::gl::blend_state_ptr blend_state_ = nullptr;

    bool needs_color_buffer_as_input_ = false;
    bool writes_only_color_buffer_ = false;
    bool enable_for_shadows_ = false;
    RenderMode rendermode_ = RenderMode::Custom;
    std::string name_ = "PipelinePass";

    std::function<void(PipelinePass&, PipelinePassDescription const&, Pipeline&)> process_ = [](PipelinePass&, PipelinePassDescription const&, Pipeline&) { return; };

  private:
    SubstitutionMap substitution_map_;
};

} // namespace gua

#endif // GUA_PIPELINE_PASS_HPP
