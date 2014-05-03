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
#include <gua/renderer/PBRUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/PBRRessource.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  void PBRUberShader::create(std::set<std::string> const& material_names)
  {
    // create ubershader factories for given material set
    UberShader::create(material_names);

  // create final ubershader that writes to gbuffer
  std::vector<ShaderProgramStage> shader_stages;
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          forward_point_rendering_vertex_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        forward_point_rendering_fragment_shader()));

  auto final_program = std::make_shared<ShaderProgram>();
  final_program->set_shaders(shader_stages);
  add_program(final_program);
  }

  ////////////////////////////////////////////////////////////////////////////////
  
  std::string const PBRUberShader::forward_point_rendering_vertex_shader() const
  {
    std::stringstream fpr_vertex;

    fpr_vertex << std::string("                      \n\
        #version 420 core                            \n\
                                                     \n\
        // input attributes                          \n\
        layout (location = 0) in vec3  in_position;  \n\
        layout (location = 1) in uint  in_r;         \n\
        layout (location = 2) in uint  in_g;         \n\
        layout (location = 3) in uint  in_b;         \n\
        layout (location = 4) in uint empty;         \n\
        layout (location = 5) in float in_radius;    \n\
        layout (location = 6) in vec3 in_normal;     \n\
                                                     \n\
        uniform mat4 gua_projection_matrix;          \n\
        uniform mat4 gua_view_matrix;                \n\
        uniform mat4 gua_model_matrix;               \n\
                                                     \n\
        out vec3 point_color;                        \n\
                                                     \n\
                                                     \n\
        void main()                                  \n\
        {                                            \n\
          gl_Position = gua_projection_matrix *      \n\
                        gua_view_matrix *            \n\
                        gua_model_matrix *           \n\
                        vec4(position,1.0);          \n\
                                                     \n\
          point_color = vec3(vec3((in_r)/255.0f,     \n\
                                  (in_g)/255.0f,     \n\
                                  (in_b)/255.0f);    \n\
        }                                            \n\
    ");

    return fpr_vertex.str();
  }

  ////////////////////////////////////////////////////////////////////////////////
  
  std::string const PBRUberShader::forward_point_rendering_fragment_shader() const
  {
    std::stringstream fpr_fragment;

    fpr_fragment << std::string("                         \n\
        #version 420 core                                 \n\
                                                          \n\
        layout(location = 0) out vec4 out_color;          \n\
                                                          \n\
                                                          \n\
                                                          \n\
        in vec3 point_color;                              \n\
                                                          \n\
                                                          \n\
        void main()                                       \n\
        {                                                 \n\
 	  out_color = vec4(point_color, 1.0);             \n\
        }                                                 \n\
    ");

    return fpr_fragment.str();
  }


  ////////////////////////////////////////////////////////////////////////////////

  bool PBRUberShader::upload_to (RenderContext const& context) const
  {
	bool upload_succeeded = UberShader::upload_to(context);

        if(context.id >= change_point_size_in_shader_state_.size() )
        {
          change_point_size_in_shader_state_.resize(context.id + 1);

          change_point_size_in_shader_state_[context.id] = context.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(true));
        }
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ GeometryUberShader::stage_mask const PBRUberShader::get_stage_mask() const
  {

    return GeometryUberShader::DRAW_STAGE;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  PBRUberShader::preframe(RenderContext const& context) const
  {
    throw std::runtime_error("PBRUberShader::preframe(): not implemented");
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  PBRUberShader::predraw(RenderContext const& ctx,
                                               std::string const& filename,
                                               std::string const& material_name,
                                               scm::math::mat4 const& model_matrix,
                                               scm::math::mat4 const& normal_matrix,
                                               Frustum const& /*frustum*/) const
  {
    throw std::runtime_error("PBRUberShader::predraw(): not implemented");
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void PBRUberShader::draw(RenderContext const& ctx,
                                           std::string const& filename,
                                           std::string const& material_name,
                                           scm::math::mat4 const& model_matrix,
                                           scm::math::mat4 const& normal_matrix,
                                           Frustum const& /*frustum*/) const
  {

    auto geometry = std::static_pointer_cast<PBRRessource>(GeometryDatabase::instance()->lookup(filename));
    auto material = MaterialDatabase::instance()->lookup(material_name);

    get_program()->use(ctx);
    {
      if (material && geometry)
      {
        set_uniform(ctx, model_matrix, "gua_model_matrix");
        set_uniform(ctx, normal_matrix, "gua_normal_matrix");

	ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);
        
        geometry->draw(ctx);

	
      }
    }
    get_program()->unuse(ctx);


  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void PBRUberShader::postdraw(RenderContext const& ctx,
    std::string const& filename,
    std::string const& material_name,
    scm::math::mat4 const& model_matrix,
    scm::math::mat4 const& normal_matrix,
    Frustum const& /*frustum*/) const
  {
    throw std::runtime_error("PBRUberShader::postdraw(): not implemented");
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void PBRUberShader::postframe(RenderContext const& context) const
  {
    throw std::runtime_error("PBRUberShader::postframe(): not implemented");
  }

}

