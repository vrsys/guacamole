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

#ifndef GUA_SHADER_PROGRAM_HPP
#define GUA_SHADER_PROGRAM_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Uniform.hpp>
#include <gua/renderer/ResourceFactory.hpp>

// external headers
#include <mutex>

#include <map>
#include <list>

namespace gua
{
/**
 *
 */
struct ShaderProgramStage
{
    ShaderProgramStage(scm::gl::shader_stage t, std::string const& src) : type(t), source(src) {}

    scm::gl::shader_stage type;
    std::string source;
};

// TODO: this class may exploit per-context resources capability

/**
 * An actual shader which can be applied to the Graphics pipeline.
 *
 * It combines data from a FragmentShader and a VertexShader in order to
 * achieve different visual appearances of the same mesh.
 */
class GUA_DLL ShaderProgram
{
  public:
  public:
    /**
     * Default constructor.
     *
     * Creates a new (invalid) shader program.
     */
    ShaderProgram();

    /**
     * Constructor.
     *
     * This method takes a vertex shader source and a fragment shader
     * source and combines them to a ShaderProgram.
     *
     * \param v_shader_source      The vertex shader source.
     * \param f_shader_source      The fragment shader source.
     * \param substitutions        Shader compile-time substitution map.
     */
    void create_from_sources(std::string const& v_shader_source, std::string const& f_shader_source, SubstitutionMap const& substitutions = SubstitutionMap());

    /**
     * Constructor.
     *
     * This method takes a vertex shader source, a geomtry shader source
     * and a fragment shader source and combines them to a ShaderProgram.
     *
     * \param v_shader_source      The vertex shader source.
     * \param g_shader_source      The geometry shader source.
     * \param f_shader_source      The fragment shader source.
     * \param substitutions        Shader compile-time substitution map.
     */
    void create_from_sources(std::string const& v_shader_source, std::string const& g_shader_source, std::string const& f_shader_source, SubstitutionMap const& substitutions = SubstitutionMap());

    /**
     *
     */
    void set_shaders(std::vector<ShaderProgramStage> const& shaders,
                     std::list<std::string> const& interleaved_stream_capture = std::list<std::string>(),
                     bool in_rasterization_discard = false,
                     SubstitutionMap const& substitutions = SubstitutionMap(),
                     bool enable_virtual_texturing = false);

    /**
     * Sets compile-time substitution map
     *
     * \param substitutions        Shader compile-time substitution map.
     */
    void set_substitutions(SubstitutionMap const& substitutions);

    /**
     * Applies this shader.
     *
     * All preceeding draw calls on the given context will be affected by
     * this shader.
     *
     * \param context             The context which should use this shader.
     */
    void use(RenderContext const& context) const;

    /**
     * Unuses the shader.
     *
     * Preceeding draw calls won't use this shader anymore.
     */
    void unuse(RenderContext const& context) const;

    /**
     * Sets an uniform value.
     *
     * Sets an uniform value of a currently used shader.
     *
     * \param uniform             The uniform to be set.
     * \param context             The context which should use this shader.
     */
    void apply_uniform(RenderContext const& context, std::string const& name, UniformValue const& uniform, unsigned position = 0) const;

    /**
     *
     */
    template <typename T>
    void set_uniform(RenderContext const& context, T const& value, std::string const& name, unsigned position = 0) const
    {
        apply_uniform(context, name, UniformValue(value), position);
    }

    /**
     *
     */
    void set_subroutine(RenderContext const& context, scm::gl::shader_stage stage, std::string const& uniform_name, std::string const& routine_name) const;

    virtual bool upload_to(RenderContext const& context) const;

    inline scm::gl::program_ptr const& get_program() const { return program_; }

    inline std::vector<ShaderProgramStage> const& get_program_stages() const { return stages_; }

  protected: // attributes
    friend class WindowBase;

    mutable scm::gl::program_ptr program_;

  private: // attributes
    mutable bool dirty_ = false;

    std::vector<ShaderProgramStage> stages_;
    std::list<std::string> interleaved_stream_capture_;
    bool in_rasterization_discard_ = false;
    SubstitutionMap substitutions_;
};

/**
 *
 */
GUA_DLL void save_to_file(ShaderProgram const& p, std::string const& directory, std::string const& name);

} // namespace gua

#endif // GUA_SHADER_PROGRAM_HPP
