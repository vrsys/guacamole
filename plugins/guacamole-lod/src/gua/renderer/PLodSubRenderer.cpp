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
#include <gua/renderer/PLodSubRenderer.hpp>
#include <gua/renderer/MaterialShader.hpp>
namespace gua
{
PLodSubRenderer::PLodSubRenderer() : shader_program_(nullptr), shaders_loaded_(false), gpu_resources_already_created_(false), current_rendertarget_width_(0), current_rendertarget_height_(0) {}

void PLodSubRenderer::_initialize_shader_program()
{
    auto new_program = std::make_shared<ShaderProgram>();
    new_program->set_shaders(shader_stages_);
    shader_program_ = new_program;
}

void PLodSubRenderer::_initialize_material_dependent_shader_programs(MaterialShader* material)
{
    auto program = std::make_shared<ShaderProgram>();

    auto smap = global_substitution_map_;
    for(const auto& i : material->generate_substitution_map())
    {
        smap[i.first] = i.second;
    }

    program->set_shaders(shader_stages_, std::list<std::string>(), false, smap);
    material_dependent_shader_programs_[material] = program;
}

std::shared_ptr<ShaderProgram> PLodSubRenderer::_get_material_dependent_shader_program(MaterialShader* material, std::shared_ptr<ShaderProgram> const& current_program, bool& program_changed)
{
    auto shader_iterator = material_dependent_shader_programs_.find(material);
    if(shader_iterator == material_dependent_shader_programs_.end())
    {
        try
        {
            _initialize_material_dependent_shader_programs(material);
            program_changed = true;
            return material_dependent_shader_programs_.at(material);
        }
        catch(std::exception& e)
        {
            Logger::LOG_WARNING << "LodPass::_get_material_program(): Cannot create material for accumulation pass program: " << e.what() << std::endl;
            return std::shared_ptr<ShaderProgram>();
        }
    }
    else
    {
        if(current_program == shader_iterator->second)
        {
            program_changed = false;
            return current_program;
        }
        else
        {
            program_changed = true;
            return shader_iterator->second;
        }
    }
}

void PLodSubRenderer::_check_for_shader_program()
{
    if(!shader_program_)
    {
        _initialize_shader_program();
    }
    assert(shader_program_);
}

void PLodSubRenderer::_register_shared_resources(gua::plod_shared_resources& shared_resources)
{
    for(auto const& attachment_ptr_entry : resource_ptrs_.attachments_)
    {
        if(!shared_resources.attachments_.count(attachment_ptr_entry.first) || shared_resources.attachments_.at(attachment_ptr_entry.first) == nullptr)
        {
            shared_resources.attachments_[attachment_ptr_entry.first] = attachment_ptr_entry.second;
        }
    }

    for(auto const& tex_buffer_ptr_entry : resource_ptrs_.tex_buffers_)
    {
        if(!shared_resources.tex_buffers_.count(tex_buffer_ptr_entry.first) || shared_resources.tex_buffers_.at(tex_buffer_ptr_entry.first) == nullptr)
        {
            shared_resources.tex_buffers_[tex_buffer_ptr_entry.first] = tex_buffer_ptr_entry.second;
        }
    }
}

} // namespace gua