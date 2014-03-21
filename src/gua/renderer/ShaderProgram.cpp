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
#include <gua/utils/logger.hpp>

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
    }
    ;

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
        WARNING("Shader stage undefined or unsupported");
      }
      ;

      save(string_utils::format_code(s.source), directory + "/" + name + file_extension);
    }
  }


////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::create_from_files(std::string const & v_file,
                                      std::string const & f_file) {

  for (auto p : programs_) {
    p.reset();

  }

  programs_.clear();
  interleaved_stream_capture_.clear();

  stages_ = { ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, v_file, false),
              ShaderProgramStage(
                  scm::gl::STAGE_FRAGMENT_SHADER, f_file, false) };
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::create_from_sources(std::string const & v_source,
                                        std::string const & f_source) {

  for (auto p : programs_) {
    p.reset();

  }

  programs_.clear();
  interleaved_stream_capture_.clear();

  stages_ = { ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, v_source, true),
              ShaderProgramStage(
                  scm::gl::STAGE_FRAGMENT_SHADER, f_source, true) };
}

////////////////////////////////////////////////////////////////////////
void ShaderProgram::set_shaders(
    std::vector<ShaderProgramStage> const & shaders,
    std::list<std::string> const & interleaved_stream_capture,
    bool in_rasterization_discard) {

  for (auto p : programs_) {
    p.reset();
  }

  programs_.clear();
  interleaved_stream_capture_.clear();

  stages_ = shaders;
  interleaved_stream_capture_ = interleaved_stream_capture;
  in_rasterization_discard_ = in_rasterization_discard;
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

//    if ( !programs_.empty() )
//    {
//      save(programs_[0]->info_log(), "program.log");
//    }

    context.render_context->bind_program(programs_[context.id]);
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::unuse(RenderContext const & context) const {
    context.render_context->reset_program();
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::apply_uniform(RenderContext const & context,
                                  UniformValueBase * uniform,
                                  std::string const & name,
                                  unsigned position) const {

    // upload to GPU if neccessary
    upload_to(context);

    uniform->apply(context, programs_[context.id], name, position);
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
    if (programs_.size() <= context.id || !programs_[context.id]) {
        std::unique_lock<std::mutex> lock(upload_mutex_);
        if (programs_.size() <= context.id) {
            programs_.resize(context.id + 1);
        }

        std::list<scm::gl::shader_ptr> shaders;
        for (auto const& s : stages_) {
            shaders.push_back(
                s.is_source
                    ? context.render_device->create_shader(s.type, s.source)
                    : context.render_device->create_shader_from_file(s.type,
                                                                     s.source));
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
            //programs_[context.id] = context.render_device->create_program (
            //shaders, capture_array, in_rasterization_discard_ );
        }

        if (!programs_[context.id]) {
          WARNING("Failed to create shaders!");

          return false;
        }

        // if (programs_[context.id]->info_log() != "")
        //   std::cout << programs_[context.id]->info_log().c_str() << std::endl;

        return true;
    }

    return false;
}

////////////////////////////////////////////////////////////////////////////////

}
