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

// external headers
#include <mutex>

#include <map>
#include <list>

namespace gua {

/**
 *
 */
struct ShaderProgramStage {
  ShaderProgramStage(scm::gl::shader_stage t,
                     std::string const& src,
                     bool is_src = true)
      : type(t), source(src), is_source(is_src) {}

  scm::gl::shader_stage type;
  std::string source;
  bool is_source;  // if false, source is a filename
};

/**
 * An actual shader which can be applied to the Graphics pipeline.
 *
 * It combines data from a FragmentShader and a VertexShader in order to
 * achieve different visual appearances of the same mesh.
 */
class GUA_DLL ShaderProgram {
 public:

  friend class NURBSUberShader;

 public:
  /**
   * Default constructor.
   *
   * Creates a new (invalid) shader program.
   */
  ShaderProgram();

  /**
   * Constructor from shaders.
   *
   * This method takes a vertex shader file and a fragment shader file
   * and combines them to a ShaderProgram.
   *
   * \param v_shader_file        The VertexShader file path.
   * \param f_shader_file        The FragmentShader file path.
   */
  void create_from_files(std::string const& v_shader_file,
                         std::string const& f_shader_file);

  /**
   * Constructor.
   *
   * This method takes a vertex shader source and a fragment shader
   * source and combines them to a ShaderProgram.
   *
   * \param v_shader_source      The vertex shader source.
   * \param f_shader_source      The fragment shader source.
   */
  void create_from_sources(std::string const& v_shader_source,
                           std::string const& f_shader_source);

  /**
   *
   */
  void set_shaders(std::vector<ShaderProgramStage> const& shaders,
                   std::list<std::string> const& interleaved_stream_capture =
                       std::list<std::string>(),
                   bool in_rasterization_discard = false);

  /**
   * Destructor
   *
   * Cleans all associated memory.
   */
  virtual ~ShaderProgram();

  /**
   *
   */
  void save_to_file(std::string const& directory,
                    std::string const& name) const;

  /**
   *
   */
  void save_log_to_file(std::string const& directory,
                        std::string const& name) const;

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
  void apply_uniform(RenderContext const& context,
                     UniformValue const& uniform,
                     std::string const& name,
                     unsigned position = 0) const;

  /**
   *
   */
  template <typename T>
  void set_uniform(RenderContext const& context,
                   T const& value,
                   std::string const& name,
                   unsigned position = 0) const {

    apply_uniform(context, UniformValue(value), name, position);
  }

  /**
   *
   */
  void set_subroutine(RenderContext const& context,
                      scm::gl::shader_stage stage,
                      std::string const& uniform_name,
                      std::string const& routine_name) const;

  virtual bool upload_to(RenderContext const& context) const;

  inline scm::gl::program_ptr const& get_program ( RenderContext const& ctx ) const { return programs_[ctx.id]; }

 protected:  // attributes

  mutable std::vector<scm::gl::program_ptr> programs_;

 private:  // attributes

  mutable std::mutex upload_mutex_;

  std::vector<ShaderProgramStage> stages_;
  std::list<std::string> interleaved_stream_capture_;
  bool in_rasterization_discard_;

};

}

#endif  // GUA_SHADER_PROGRAM_HPP
