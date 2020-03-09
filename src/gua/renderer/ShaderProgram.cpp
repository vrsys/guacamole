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

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

ShaderProgram::ShaderProgram() : program_(), stages_(), interleaved_stream_capture_() {}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::create_from_sources(std::string const& v_source, std::string const& f_source, SubstitutionMap const& substitutions)
{
    program_.reset();
    dirty_ = true;

    interleaved_stream_capture_.clear();
    in_rasterization_discard_ = false;
    substitutions_ = substitutions;

    stages_ = {ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, v_source), ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_source)};
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::create_from_sources(std::string const& v_source, std::string const& g_source, std::string const& f_source, SubstitutionMap const& substitutions)
{
    program_.reset();
    dirty_ = true;

    interleaved_stream_capture_.clear();
    in_rasterization_discard_ = false;
    substitutions_ = substitutions;

    stages_ = {ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, v_source), ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, g_source), ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, f_source)};
}

////////////////////////////////////////////////////////////////////////
void ShaderProgram::set_shaders(std::vector<ShaderProgramStage> const& shaders,
                                std::list<std::string> const& interleaved_stream_capture,
                                bool in_rasterization_discard,
                                bool in_early_fragment_test,
                                SubstitutionMap const& substitutions,
                                bool enable_virtual_texturing)
{
    program_.reset();
    dirty_ = true;

    interleaved_stream_capture_.clear();

    stages_ = shaders;
    interleaved_stream_capture_ = interleaved_stream_capture;
    in_rasterization_discard_ = in_rasterization_discard;
    substitutions_ = substitutions;
    substitutions_["enable_virtual_texturing"] = enable_virtual_texturing ? "1" : "0";
    substitutions_["enable_early_fragment_test"] = in_early_fragment_test ? "1" : "0";
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::set_substitutions(SubstitutionMap const& substitutions)
{
    if(substitutions_ != substitutions)
    {
        substitutions_ = substitutions;
        program_.reset();
        dirty_ = true;
    }
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::use(RenderContext const& context) const
{
    // upload to GPU if neccessary
    upload_to(context);
    context.render_context->bind_program(program_);
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::unuse(RenderContext const& context) const { context.render_context->reset_program(); }

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::apply_uniform(RenderContext const& context, std::string const& name, UniformValue const& uniform, unsigned position) const
{
    // upload to GPU if neccessary
    upload_to(context);
    uniform.apply(context, name, program_, position);
}

////////////////////////////////////////////////////////////////////////////////

void ShaderProgram::set_subroutine(RenderContext const& context, scm::gl::shader_stage stage, std::string const& uniform_name, std::string const& routine_name) const
{
    // upload to GPU if neccessary
    upload_to(context);

    program_->uniform_subroutine(stage, uniform_name, routine_name);
}

////////////////////////////////////////////////////////////////////////////////

bool ShaderProgram::upload_to(RenderContext const& context) const
{
    if(!program_ || dirty_)
    {
        std::list<scm::gl::shader_ptr> shaders;
        ResourceFactory factory;

        for(auto const& s : stages_)
        {
            auto source = factory.resolve_substitutions(s.source, substitutions_);
            shaders.push_back(context.render_device->create_shader(s.type, source));
        }

        if(interleaved_stream_capture_.empty())
        {
            program_ = context.render_device->create_program(shaders);
        }
        else
        {
            scm::gl::interleaved_stream_capture capture_array(interleaved_stream_capture_.front());
            for(auto const& k : interleaved_stream_capture_)
                capture_array(k);

            program_ = context.render_device->create_program(shaders, capture_array, in_rasterization_discard_);
        }

        dirty_ = false;

        if(!program_)
        {
            Logger::LOG_WARNING << "Failed to create shaders!" << std::endl;
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////

void save_to_file(ShaderProgram const& p, std::string const& directory, std::string const& name)
{
    auto save = [](std::string const& content, std::string const& file) {
        gua::TextFile text(file);
        // text.set_content(string_utils::format_code(content));
        text.set_content(content);
        text.save(true);
    };

    for(auto const& s : p.get_program_stages())
    {
        std::string file_extension;
        switch(s.type)
        {
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

} // namespace gua
