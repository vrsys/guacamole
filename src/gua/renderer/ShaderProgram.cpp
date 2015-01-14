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

// class header
#include <gua/renderer/ShaderProgram.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Uniform.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

ShaderProgram::ShaderProgram()
    : programs_(), upload_mutex_(), stages_(), interleaved_stream_capture_() {}

////////////////////////////////////////////////////////////////////////////////

ShaderProgram::~ShaderProgram() {}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::save_to_file(std::string const& directory,
                                 std::string const& name) const {

  auto save = [](std::string const & content, std::string const & file) {
    gua::TextFile text(file);
    //text.set_content(string_utils::format_code(content));
    text.set_content(content);
    text.save(true);
  };

  for (auto const& s : stages_) {
    std::string file_extension;
    switch (s.type) {
      case scm::gl::STAGE_VERTEX_SHADER:
        file_extension = ".vert";
        break;
      case scm::gl::STAGE_GEOMETRY_SHADER:
        file_extension = ".geom";
        break;
      case scm::gl::STAGE_FRAGMENT_SHADER:
        file_extension = ".frag";
        break;
      case scm::gl::STAGE_TESS_EVALUATION_SHADER:
        file_extension = ".teval";
        break;
      case scm::gl::STAGE_TESS_CONTROL_SHADER:
        file_extension = ".tctrl";
        break;
      default:
        Logger::LOG_WARNING << "Shader stage undefined or unsupported" << std::endl;
    };

    save(string_utils::format_code(s.source), directory + "/" + name + file_extension);
  }
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::create_from_sources(std::string const & v_source,
                                        std::string const & f_source,
                                        SubstitutionMap const& substitutions) {

  for (auto p : programs_) {
    p.reset();
  }

  programs_.clear();
  interleaved_stream_capture_.clear();
  in_rasterization_discard_ = false;
  substitutions_ = substitutions;
  dirty_ = true;

  stages_ = { ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_source),
              ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_source) };
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::create_from_sources(std::string const & v_source,
                                        std::string const & g_source,
                                        std::string const & f_source,
                                        SubstitutionMap const& substitutions) {

  for (auto p : programs_) {
    p.reset();

  }

  programs_.clear();
  interleaved_stream_capture_.clear();
  in_rasterization_discard_ = false;
  substitutions_ = substitutions;
  dirty_ = true;

  stages_ = { ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,   v_source),
              ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_source),
              ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_source) };
}

////////////////////////////////////////////////////////////////////////
void ShaderProgram::set_shaders(
    std::vector<ShaderProgramStage> const & shaders,
    std::list<std::string> const & interleaved_stream_capture,
    bool in_rasterization_discard,
                           SubstitutionMap const& substitutions) {

  for (auto p : programs_) {
    p.reset();
  }

  programs_.clear();
  interleaved_stream_capture_.clear();

  stages_ = shaders;
  interleaved_stream_capture_ = interleaved_stream_capture;
  in_rasterization_discard_ = in_rasterization_discard;
  substitutions_ = substitutions;
  dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::set_substitutions(SubstitutionMap const& substitutions) {
  if (substitutions_ != substitutions) {
    substitutions_ = substitutions;
    dirty_ = true;
  }
}


////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::use(RenderContext const & context) const {

  // upload to GPU if neccessary
  upload_to(context);

  auto save = [](std::string const & content, std::string const & file) {
    gua::TextFile text(file);
    text.set_content(string_utils::format_code(content));
    text.save();
  };

  //if ( !programs_.empty() )
  //{
  //  save(programs_[0]->info_log(), "program.log");
  //}

  context.render_context->bind_program(programs_[context.id]);
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::unuse(RenderContext const & context) const {
  context.render_context->reset_program();
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::apply_uniform(RenderContext const & context,
                                  std::string const& name,
                                  UniformValue const& uniform,
                                  unsigned position) const {

  // upload to GPU if neccessary
  upload_to(context);

  uniform.apply(context, name, programs_[context.id], position);
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::set_subroutine(RenderContext const & context,
                                   scm::gl::shader_stage stage,
                                   std::string const & uniform_name,
                                   std::string const & routine_name) const {

  // upload to GPU if neccessary
  upload_to(context);

  programs_[context.id]
      ->uniform_subroutine(stage, uniform_name, routine_name);
}

////////////////////////////////////////////////////////////////////////////////

bool ShaderProgram::upload_to(RenderContext const & context) const {

  if (   programs_.size() <= context.id
      || !programs_[context.id]
      || dirty_) {
    std::unique_lock<std::mutex> lock(upload_mutex_);
    if (programs_.size() <= context.id) {
      programs_.resize(context.id + 1);
    }

    std::list<scm::gl::shader_ptr> shaders;
    ResourceFactory factory;
    for (auto const& s : stages_) {
      auto source = factory.resolve_substitutions(s.source, substitutions_);
      shaders.push_back(context.render_device->create_shader(s.type, source));
    }

    if (interleaved_stream_capture_.empty()) {
      programs_[context.id] =
          context.render_device->create_program(shaders);
    } else {
      scm::gl::interleaved_stream_capture capture_array(
          interleaved_stream_capture_.front());
      auto k = interleaved_stream_capture_.begin();
      while (++k != interleaved_stream_capture_.end()) {
        capture_array(*k);
      }

      programs_[context.id] = context.render_device->create_program(
          shaders, capture_array, in_rasterization_discard_);
    }

    dirty_ = false;

    if (!programs_[context.id]) {
      Logger::LOG_WARNING << "Failed to create shaders!" << std::endl;

      return false;
    }
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

}
