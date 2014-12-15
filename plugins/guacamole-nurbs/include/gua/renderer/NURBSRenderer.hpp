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
#include <gua/renderer/ProgramFactory.hpp>

namespace gua {

  class MaterialShader;
  class ShaderProgram;

  class GUA_NURBS_DLL NURBSRenderer {

  public:

    NURBSRenderer();
    ~NURBSRenderer();

    void render(Pipeline& pipe, PipelinePassDescription const& desc);

    void reload_programs();

  private:  // auxiliary methods

    void        _load_shaders();
    void        _initialize_pre_tesselation_program();
    void        _initialize_tesselation_program(MaterialShader*);
    void        _reset();

    std::string _transform_feedback_vertex_shader() const;
    std::string _transform_feedback_geometry_shader() const;
    std::string _transform_feedback_tess_control_shader() const;
    std::string _transform_feedback_tess_evaluation_shader() const;

    std::string _final_vertex_shader() const;
    std::string _final_tess_control_shader() const;
    std::string _final_tess_evaluation_shader() const;
    std::string _final_geometry_shader() const;
    std::string _final_fragment_shader() const;

    std::string _raycast_vertex_shader() const;
    std::string _raycast_fragment_shader() const;

  private:  // attributes

    unsigned                                            current_modcount_;
    ProgramFactory                                      factory_;
                                                        
    // CPU Ressources                                   
    std::vector<ShaderProgramStage>                     pre_tesselation_shader_stages_;
    std::list<std::string>                              pre_tesselation_interleaved_stream_capture_;
    std::map<scm::gl::shader_stage, std::string>        tesselation_shader_stages_;                 
    std::map<scm::gl::shader_stage, std::string>        raycasting_shader_stages_;

    // GPU Ressources
    std::shared_ptr<ShaderProgram>                                      pre_tesselation_program_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> tesselation_programs_;
    std::unordered_map<MaterialShader*, std::shared_ptr<ShaderProgram>> raycasting_programs_;

  };

}

#endif  // GUA_NURBS_RENDERER_HPP
