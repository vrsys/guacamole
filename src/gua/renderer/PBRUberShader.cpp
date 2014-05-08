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
#include <gua/renderer/Window.hpp>
#include <gua/renderer/PBRRessource.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////

  void PBRUberShader::create(std::set<std::string> const& material_names)
  {
     UberShader::create(material_names);

    // create depth shader
    std::vector<ShaderProgramStage> point_forward_pass_stages;
    point_forward_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          accumulation_pass_vertex_shader()));
    point_forward_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        accumulation_pass_fragment_shader()));

    auto point_forward_pass_program = std::make_shared<ShaderProgram>();
    point_forward_pass_program->set_shaders(point_forward_pass_stages);
    add_program(point_forward_pass_program);

    // create final shader
    std::vector<ShaderProgramStage> normalization_pass_stages;
    normalization_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, normalization_pass_vertex_shader()));
    normalization_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, normalization_pass_fragment_shader()));

    auto normalization_pass_program = std::make_shared<ShaderProgram>();
    normalization_pass_program->set_shaders(normalization_pass_stages);
    add_program(normalization_pass_program);
  }






  ////////////////////////////////////////////////////////////////////////////////
  

/*
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
                        vec4(in_position,1.0);          \n\
                                                     \n\
          point_color = vec3((in_r)/255.0f,     \n\
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
       layout(location = 0) out vec4 out_color;        \n\
                                                          \n\
        in vec3 point_color;                              \n\
                                                          \n\
                                                          \n\
        void main()                                       \n\
        {                                                 \n\
          out_color = vec4(1.0,0.0,0.0,1.0);              \n\
        }                                                 \n\
    ");

    return fpr_fragment.str();
  }


*/

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



std::string const PBRUberShader::accumulation_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_point_forward_vert)
    );
  return vertex_shader;
}







////////////////////////////////////////////////////////////////////////////////

std::string const PBRUberShader::accumulation_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_point_forward_frag)
    );

  return fragment_shader;
}


  ////////////////////////////////////////////////////////////////////////////////



std::string const PBRUberShader::normalization_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_blend_pass_vert)
  );

/*
  // material specific uniforms
  string_utils::replace(vertex_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

*/
  // output
  string_utils::replace(vertex_shader, "@output_definition",
    vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true));

/*
  // print material specific methods
  string_utils::replace(vertex_shader, "@material_methods",
    UberShader::print_material_methods(*vshader_factory_));

  // print main switch(es)
  string_utils::replace(vertex_shader, "@material_switch",
    UberShader::print_material_switch(*vshader_factory_));
*/
  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const PBRUberShader::normalization_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_blend_pass_frag)
    );

  std::string apply_pbr_color = fshader_factory_->get_output_mapping().get_output_string("gua_pbr", "gua_pbr_output_color");
  apply_pbr_color += " = output_color;\n";

  std::string apply_pbr_normal = fshader_factory_->get_output_mapping().get_output_string("gua_pbr", "gua_normal");

  apply_pbr_normal += " = output_normal;\n";

  string_utils::replace(fragment_shader, "@apply_pbr_color", apply_pbr_color);
  string_utils::replace(fragment_shader, "@apply_pbr_normal", apply_pbr_normal);

  // input from vertex shader
  string_utils::replace(fragment_shader, "@input_definition",
    vshader_factory_->get_output_mapping().get_gbuffer_output_definition(true, true));

  // material specific uniforms
  string_utils::replace(fragment_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(fragment_shader, "@output_definition",
    get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  // print material specific methods
  string_utils::replace(fragment_shader, "@material_methods",
    UberShader::print_material_methods(*fshader_factory_));

  // print main switch(es)
  string_utils::replace(fragment_shader, "@material_switch",
    UberShader::print_material_switch(*fshader_factory_));

  return fragment_shader;
}




  ////////////////////////////////////////////////////////////////////////////////

bool PBRUberShader::upload_to (RenderContext const& context) const
{
  bool upload_succeeded = UberShader::upload_to(context);

  assert(context.render_window->config.get_stereo_mode() == StereoMode::MONO ||
    ((context.render_window->config.get_left_resolution()[0] == context.render_window->config.get_right_resolution()[0]) &&
    (context.render_window->config.get_left_resolution()[1] == context.render_window->config.get_right_resolution()[1])));

  // initialize Texture Arrays (kinect depths & colors)
  if ( context.id >= point_forward_color_result_.size() ||
       context.id >= point_forward_depth_result_.size()) 
  {
    point_forward_color_result_.resize(context.id + 1);
    point_forward_depth_result_.resize(context.id + 1);


    point_forward_color_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_RGBA_32F,
      1,
      1,
      1
      );

    point_forward_depth_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_D32F,
      1,
      1,
      1
      );
  }

  if (context.id >= fullscreen_quad_.size()) {
    fullscreen_quad_.resize(context.id + 1);
    fullscreen_quad_[context.id].reset(new scm::gl::quad_geometry(context.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
  }

  if (context.id >= point_forward_result_fbo_.size()) {
    point_forward_result_fbo_.resize(context.id + 1);
    point_forward_result_fbo_[context.id] = context.render_device->create_frame_buffer();
  }

  if (context.id >= linear_sampler_state_.size()) {
    linear_sampler_state_.resize(context.id + 1);
    linear_sampler_state_[context.id] = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
  }

  if (context.id >= nearest_sampler_state_.size()) {
    nearest_sampler_state_.resize(context.id + 1);
    nearest_sampler_state_[context.id] = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
  }

  if (context.id >= depth_stencil_state_.size()) {
    depth_stencil_state_.resize(context.id + 1);
    depth_stencil_state_[context.id] = context.render_device->create_depth_stencil_state(false, true, scm::gl::COMPARISON_LESS);
  }

  if (context.id >= change_point_size_in_shader_state_.size()){
    change_point_size_in_shader_state_.resize(context.id + 1);
    change_point_size_in_shader_state_[context.id] = context.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(true));
  }


  
  return upload_succeeded;
}

////////////////////////////////////////////////////////////////////////////////

void PBRUberShader::draw(RenderContext const& ctx,
                             std::string const& file_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& /*frustum*/) const
{
  if (!GeometryDatabase::instance()->is_supported(file_name) || 
      !MaterialDatabase::instance()->is_supported(material_name)) {
    gua::Logger::LOG_WARNING << "PBRUberShader::draw(): No such video or material." << file_name << ", " << material_name << std::endl;
    return;
  } 

  auto pbr_ressource     = std::static_pointer_cast<PBRRessource>(GeometryDatabase::instance()->lookup(file_name));
  auto material          = MaterialDatabase::instance()->lookup(material_name);

  if (!material || !pbr_ressource) {
    gua::Logger::LOG_WARNING << "PBRUberShader::draw(): Invalid video or material." << std::endl;
    return;
  }

  // make sure ressources are on the GPU
  upload_to(ctx);
  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    //ctx.render_context->bind_texture(video3d_ressource->depth_array(ctx), nearest_sampler_state_[ctx.id], 0);
    //get_program(warp_pass)->get_program(ctx)->uniform_sampler("depth_video3d_texture", 0);

    auto ds_state = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    ctx.render_context->set_depth_stencil_state(ds_state);



    // pre passes
    // forward rendering, will later be changed to be actually pass 2 (accumulation)

      // configure fbo
      point_forward_result_fbo_[ctx.id]->clear_attachments();
      point_forward_result_fbo_[ctx.id]->attach_depth_stencil_buffer(point_forward_depth_result_[ctx.id]);
      point_forward_result_fbo_[ctx.id]->attach_color_buffer(0, point_forward_color_result_[ctx.id]);
      
      // bind and clear fbo
      ctx.render_context->set_frame_buffer(point_forward_result_fbo_[ctx.id]);
      ctx.render_context->clear_depth_stencil_buffer(point_forward_result_fbo_[ctx.id]);
      ctx.render_context->clear_color_buffer(point_forward_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
       

      get_program(point_forward_pass)->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
      get_program(point_forward_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");

      if (material && pbr_ressource)
      {

        get_program(point_forward_pass)->use(ctx);
        {
          pbr_ressource->draw(ctx);
        }
        get_program(point_forward_pass)->unuse(ctx);
      }

      ctx.render_context->reset_framebuffer();
    }
  





  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    // second pass
    get_program(quad_pass)->use(ctx);
    {
      if (material /*&& video3d_ressource*/)
      {
        get_program(quad_pass)->set_uniform(ctx, material->get_id(), "gua_material_id");
        get_program(quad_pass)->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
        get_program(quad_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");


        get_program(quad_pass)->set_uniform(ctx, int(material_name == default_pbr_material_name()) , "using_default_pbr_material");

        ctx.render_context->bind_texture(point_forward_color_result_[ctx.id], nearest_sampler_state_[ctx.id], 0);
        get_program(quad_pass)->get_program(ctx)->uniform_sampler("color_texture", 0);

        fullscreen_quad_[ctx.id]->draw(ctx.render_context);
      }
    }
    get_program(quad_pass)->unuse(ctx);
  }
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


////////////////////////////////////////////////////////////////////////////////

  std::string const PBRUberShader::default_pbr_material_name() const
  {
    return "gua_pbr";
  }

}

