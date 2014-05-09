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
#include <gua/guacamole.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/Window.hpp>
#include <gua/renderer/PBRRessource.hpp>
#include <gua/databases.hpp>

#include <gua/databases/MaterialDatabase.hpp>

#include <gua/utils/Logger.hpp>


namespace gua {


////////////////////////////////////////////////////////////////////////////////

PBRUberShader::PBRUberShader()
  : GeometryUberShader(), near_plane_value_(0.0f), height_divided_by_top_minus_bottom_(0.0f)
{
  if (!MaterialDatabase::instance()->is_supported(default_pbr_material_name()))
  {
    create_resource_material(default_pbr_material_name(),
      Resources::materials_gua_pbr_gsd,
      Resources::materials_gua_pbr_gmd);
  }
}


  ////////////////////////////////////////////////////////////////////////////////

  void PBRUberShader::create(std::set<std::string> const& material_names)
  {
     UberShader::create(material_names);

    // create depth pass shader
    std::vector<ShaderProgramStage> depth_pass_stages;
    depth_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          depth_pass_vertex_shader()));
    depth_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        depth_pass_fragment_shader()));

    auto depth_pass_program = std::make_shared<ShaderProgram>();
    depth_pass_program->set_shaders(depth_pass_stages);
    add_program(depth_pass_program);

    // create depth shader
    std::vector<ShaderProgramStage> accumulation_pass_stages;
    accumulation_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          accumulation_pass_vertex_shader()));
    accumulation_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        accumulation_pass_fragment_shader()));

    auto accumulation_pass_program = std::make_shared<ShaderProgram>();
    accumulation_pass_program->set_shaders(accumulation_pass_stages);
    add_program(accumulation_pass_program);

    // create final shader
    std::vector<ShaderProgramStage> normalization_pass_stages;
    normalization_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, normalization_pass_vertex_shader()));
    normalization_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, normalization_pass_fragment_shader()));

    auto normalization_pass_program = std::make_shared<ShaderProgram>();
    normalization_pass_program->set_shaders(normalization_pass_stages);
    add_program(normalization_pass_program);
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



std::string const PBRUberShader::depth_pass_vertex_shader() const
{

  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p01_depth_vert)
    );

  return vertex_shader;
}



////////////////////////////////////////////////////////////////////////////////

std::string const PBRUberShader::depth_pass_fragment_shader() const
{

  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p01_depth_frag)
    );

  return fragment_shader;
}



////////////////////////////////////////////////////////////////////////////////



std::string const PBRUberShader::accumulation_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p02_accumulation_vert)
    );
  return vertex_shader;
}



////////////////////////////////////////////////////////////////////////////////

std::string const PBRUberShader::accumulation_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p02_accumulation_frag)
    );

  return fragment_shader;
}


  ////////////////////////////////////////////////////////////////////////////////



std::string const PBRUberShader::normalization_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_vert)
  );


  // material specific uniforms
  string_utils::replace(vertex_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());


  // output
  string_utils::replace(vertex_shader, "@output_definition",
    vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true));

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const PBRUberShader::normalization_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_frag)
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


  return fragment_shader;
}




  ////////////////////////////////////////////////////////////////////////////////

bool PBRUberShader::upload_to (RenderContext const& context) const
{
  bool upload_succeeded = UberShader::upload_to(context);

  assert(context.render_window->config.get_stereo_mode() == StereoMode::MONO ||
    ((context.render_window->config.get_left_resolution()[0] == context.render_window->config.get_right_resolution()[0]) &&
    (context.render_window->config.get_left_resolution()[1] == context.render_window->config.get_right_resolution()[1])));



  // initialize attachments for depth pass
  if ( context.id >= depth_pass_color_result_.size() ||
       context.id >= depth_pass_depth_result_.size()) 
  {
    depth_pass_color_result_.resize(context.id + 1);
    depth_pass_depth_result_.resize(context.id + 1);


    depth_pass_color_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_RGBA_32F,
      1,
      1,
      1
      );

    depth_pass_depth_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_D32F,
      1,
      1,
      1
      );
  }

  // initialize attachments for accumulation pass
  if ( context.id >= accumulation_pass_color_result_.size() ||
       context.id >= accumulation_pass_depth_result_.size()) 
  {
    accumulation_pass_color_result_.resize(context.id + 1);
    accumulation_pass_depth_result_.resize(context.id + 1);


    accumulation_pass_color_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_RGBA_32F,
      1,
      1,
      1
      );

    accumulation_pass_depth_result_[context.id] = context.render_device->create_texture_2d(
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

  if (context.id >= depth_pass_result_fbo_.size()) {
    depth_pass_result_fbo_.resize(context.id + 1);
    depth_pass_result_fbo_[context.id] = context.render_device->create_frame_buffer();
  }

  if (context.id >= accumulation_pass_result_fbo_.size()) {
    accumulation_pass_result_fbo_.resize(context.id + 1);
    accumulation_pass_result_fbo_[context.id] = context.render_device->create_frame_buffer();
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
                             Frustum const& frustum) const
{
  if (!GeometryDatabase::instance()->is_supported(file_name) || 
      !MaterialDatabase::instance()->is_supported(material_name)) {
    gua::Logger::LOG_WARNING << "PBRUberShader::draw(): No such pbr ressource or material." << file_name << ", " << material_name << std::endl;
    return;
  } 

  auto pbr_ressource     = std::static_pointer_cast<PBRRessource>(GeometryDatabase::instance()->lookup(file_name));
  auto material          = MaterialDatabase::instance()->lookup(material_name);

  if (!material || !pbr_ressource) {
    gua::Logger::LOG_WARNING << "PBRUberShader::draw(): Invalid pbr ressource or material." << std::endl;
    return;
  }

  // make sure ressources are on the GPU
  upload_to(ctx);


/*
   // pre passes:
   /////////////////////
   // begin of depth pass (first)
  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);

    auto ds_state = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    ctx.render_context->set_depth_stencil_state(ds_state);






      // configure fbo
      depth_pass_result_fbo_[ctx.id]->clear_attachments();
      depth_pass_result_fbo_[ctx.id]->attach_depth_stencil_buffer(depth_pass_depth_result_[ctx.id]);
      depth_pass_result_fbo_[ctx.id]->attach_color_buffer(0, depth_pass_color_result_[ctx.id]);
      
     
      // bind and clear fbo
      ctx.render_context->set_frame_buffer(depth_pass_result_fbo_[ctx.id]);
      ctx.render_context->clear_depth_stencil_buffer(depth_pass_result_fbo_[ctx.id]);
      ctx.render_context->clear_color_buffer(depth_pass_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));  


      gua::math::mat4 const& projection_matrix = frustum.get_projection(); 

      float   near_plane_value = frustum.get_clip_near();
      float   top_plane_value = near_plane_value * (1.0 + projection_matrix[9]) / projection_matrix[5];
      float bottom_plane_value = near_plane_value * (projection_matrix[9] - 1.0) / projection_matrix[5];
      float height_divided_by_top_minus_bottom = 800.0 / (top_plane_value - bottom_plane_value);



      get_program(depth_pass)->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
      get_program(depth_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");

      get_program(depth_pass)->set_uniform(ctx, height_divided_by_top_minus_bottom, "height_divided_by_top_minus_bottom");
      get_program(depth_pass)->set_uniform(ctx, near_plane_value, "near_plane");

      if (material && pbr_ressource)
      {

        get_program(depth_pass)->use(ctx);
        {
          pbr_ressource->draw(ctx);
        }
        get_program(depth_pass)->unuse(ctx);
      }

      ctx.render_context->reset_framebuffer();
    }
*/
   // begin of accumulation pass (first)

  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);

    //!!!!!!!!!!!!EXCHANGE THIS WITH DEPH TEST NONE AND PUT COLOR ACCUMULATION ALSO SOMEWHERE
    auto ds_state = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    ctx.render_context->set_depth_stencil_state(ds_state);


      // configure fbo
      accumulation_pass_result_fbo_[ctx.id]->clear_attachments();
      accumulation_pass_result_fbo_[ctx.id]->attach_depth_stencil_buffer(accumulation_pass_depth_result_[ctx.id]);
      accumulation_pass_result_fbo_[ctx.id]->attach_color_buffer(0, accumulation_pass_color_result_[ctx.id]);
      
     
      // bind and clear fbo
      ctx.render_context->set_frame_buffer(accumulation_pass_result_fbo_[ctx.id]);
      ctx.render_context->clear_depth_stencil_buffer(accumulation_pass_result_fbo_[ctx.id]);
      ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));  


      gua::math::mat4 const& projection_matrix = frustum.get_projection(); 

      //put near_plane_value and height_divided_by_top_minus_bottom as member variables and get these two only 1 per render frame
      float   near_plane_value = frustum.get_clip_near();
      float    top_plane_value = near_plane_value * (1.0 + projection_matrix[9]) / projection_matrix[5];
      float bottom_plane_value = near_plane_value * (projection_matrix[9] - 1.0) / projection_matrix[5];
      float height_divided_by_top_minus_bottom = 800.0 / (top_plane_value - bottom_plane_value);

      //std::cout << "hdbtmb: "<< height_divided_by_top_minus_bottom;

      get_program(accumulation_pass)->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
      get_program(accumulation_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");

      get_program(accumulation_pass)->set_uniform(ctx, height_divided_by_top_minus_bottom, "height_divided_by_top_minus_bottom");
      get_program(accumulation_pass)->set_uniform(ctx, near_plane_value, "near_plane");

      if (material && pbr_ressource)
      {

        get_program(accumulation_pass)->use(ctx);
        {
          pbr_ressource->draw(ctx);
        }
        get_program(accumulation_pass)->unuse(ctx);
      }

      ctx.render_context->reset_framebuffer();
    }
  





  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    auto ds_state = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    ctx.render_context->set_depth_stencil_state(ds_state);

    // second pass
    get_program(normalization_pass)->use(ctx);
    {
      if (material /*&& video3d_ressource*/)
      {
        get_program(normalization_pass)->set_uniform(ctx, material->get_id(), "gua_material_id");
        get_program(normalization_pass)->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
        get_program(normalization_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");


        get_program(normalization_pass)->set_uniform(ctx, int(material_name == default_pbr_material_name()) , "using_default_pbr_material");


 

        ctx.render_context->bind_texture(accumulation_pass_color_result_[ctx.id], nearest_sampler_state_[ctx.id], 0);
        get_program(normalization_pass)->get_program(ctx)->uniform_sampler("color_texture", 0);


        fullscreen_quad_[ctx.id]->draw(ctx.render_context);
      }
    }
    get_program(normalization_pass)->unuse(ctx);
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

