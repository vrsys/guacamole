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
{}


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

    // create accumulation pass shader
    std::vector<ShaderProgramStage> accumulation_pass_stages;
    accumulation_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          accumulation_pass_vertex_shader()));
    accumulation_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        accumulation_pass_fragment_shader()));

    auto accumulation_pass_program = std::make_shared<ShaderProgram>();
    accumulation_pass_program->set_shaders(accumulation_pass_stages);
    add_program(accumulation_pass_program);

    // create normalization pass shader
    std::vector<ShaderProgramStage> normalization_pass_stages;
    normalization_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, normalization_pass_vertex_shader()));
    normalization_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, normalization_pass_fragment_shader()));

    auto normalization_pass_program = std::make_shared<ShaderProgram>();
    normalization_pass_program->set_shaders(normalization_pass_stages);
    add_program(normalization_pass_program);

    // create reconstruction pass shader
    std::vector<ShaderProgramStage> reconstruction_pass_stages;
    reconstruction_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, reconstruction_pass_vertex_shader()));
    reconstruction_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, reconstruction_pass_fragment_shader()));

    auto reconstruction_pass_program = std::make_shared<ShaderProgram>();
    reconstruction_pass_program->set_shaders(reconstruction_pass_stages);
    add_program(reconstruction_pass_program);
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

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const PBRUberShader::normalization_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_frag)
    );

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////



std::string const PBRUberShader::reconstruction_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p04_reconstruction_vert)
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

std::string const PBRUberShader::reconstruction_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p04_reconstruction_frag)
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
  if (context.id >= depth_pass_log_depth_result_.size()) 
  {

    depth_pass_log_depth_result_.resize(context.id + 1);


    depth_pass_log_depth_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_D32,
      1,
      1,
      1
      );
  }

  if (context.id >= depth_pass_linear_depth_result_.size()) 
  {

    depth_pass_linear_depth_result_.resize(context.id + 1);


    depth_pass_linear_depth_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_R_32F,
      1,
      1,
      1
      );
  }

  // initialize attachments for accumulation pass
  if (context.id >= accumulation_pass_color_result_.size())
  {
    accumulation_pass_color_result_.resize(context.id + 1);


    accumulation_pass_color_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_RGBA_32F,
      1,
      1,
      1
      );

  }

  // initialize attachment for normalization pass
  if (context.id >= normalization_pass_color_result_.size()) 
  {

    normalization_pass_color_result_.resize(context.id + 1);


    normalization_pass_color_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_RGB_8,
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
    
    // configure depth FBO
    depth_pass_result_fbo_[context.id]->clear_attachments();   
    depth_pass_result_fbo_[context.id]->attach_depth_stencil_buffer(depth_pass_log_depth_result_[context.id]);
    depth_pass_result_fbo_[context.id]->attach_color_buffer(0, depth_pass_linear_depth_result_[context.id]);
  }

  if (context.id >= accumulation_pass_result_fbo_.size()) {
    accumulation_pass_result_fbo_.resize(context.id + 1);
    accumulation_pass_result_fbo_[context.id] = context.render_device->create_frame_buffer();

    // configure accumulation FBO
    accumulation_pass_result_fbo_[context.id]->clear_attachments();
    accumulation_pass_result_fbo_[context.id]->attach_color_buffer(0, accumulation_pass_color_result_[context.id]);
  }

  if (context.id >= normalization_pass_result_fbo_.size()) {
    normalization_pass_result_fbo_.resize(context.id + 1);
    normalization_pass_result_fbo_[context.id] = context.render_device->create_frame_buffer();

    // configure normalization FBO
    normalization_pass_result_fbo_[context.id]->clear_attachments();
    normalization_pass_result_fbo_[context.id]->attach_color_buffer(0, normalization_pass_color_result_[context.id]);
  }

  if (context.id >= linear_sampler_state_.size()) {
    linear_sampler_state_.resize(context.id + 1);
    linear_sampler_state_[context.id] = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
  }

  if (context.id >= no_depth_test_depth_stencil_state_.size()) {
    no_depth_test_depth_stencil_state_.resize(context.id + 1);
    no_depth_test_depth_stencil_state_[context.id] = context.render_device->create_depth_stencil_state(false, false, scm::gl::COMPARISON_ALWAYS);
  }

  if (context.id >= color_accumulation_state_.size()) {
    color_accumulation_state_.resize(context.id + 1);
    color_accumulation_state_[context.id] = context.render_device->create_blend_state(true, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::FUNC_ONE, scm::gl::EQ_FUNC_ADD, scm::gl::EQ_FUNC_ADD);
  }


  if (context.id >= change_point_size_in_shader_state_.size()){
    change_point_size_in_shader_state_.resize(context.id + 1);
    change_point_size_in_shader_state_[context.id] = context.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, false, false, 0.0, false, false, scm::gl::point_raster_state(true));
  }

  if (context.id >= material_id_.size()){
    material_id_.resize(context.id + 1);
    material_id_[context.id] = 0;
  }

  if (context.id >= near_plane_value_.size()){
    near_plane_value_.resize(context.id + 1);
    near_plane_value_[context.id] = 0;
  }

  if (context.id >= height_divided_by_top_minus_bottom_.size()){
    height_divided_by_top_minus_bottom_.resize(context.id + 1);
    height_divided_by_top_minus_bottom_[context.id] = 0;
  }

  if (context.id >= render_window_dims_.size()){
    render_window_dims_.resize(context.id + 1);
    render_window_dims_[context.id] = context.render_window->config.get_left_resolution();
  }

  if (context.id >= context_guard_.size()){
    context_guard_.resize(context.id + 1);
  }

  if (context.id >=  last_geometry_state_.size())
  {
    last_geometry_state_.resize(context.id + 1);
    last_geometry_state_[context.id] = invalid_state;
  }


  return upload_succeeded;
}






  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ GeometryUberShader::stage_mask const PBRUberShader::get_stage_mask() const
  {

    return GeometryUberShader::PRE_FRAME_STAGE | GeometryUberShader::PRE_DRAW_STAGE | GeometryUberShader::POST_DRAW_STAGE | GeometryUberShader::POST_FRAME_STAGE;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  PBRUberShader::preframe(RenderContext const& ctx) const
  {


      //scm::gl::context_all_guard guard(ctx.render_context);
      upload_to(ctx);

      last_geometry_state_[ctx.id] = pre_frame_state;


      context_guard_[ctx.id] = std::make_shared<scm::gl::context_all_guard>(ctx.render_context); 



      {

      // clear depth FBOs depth attachments

       ctx.render_context->clear_depth_stencil_buffer(depth_pass_result_fbo_[ctx.id]);
       ctx.render_context->clear_color_buffer(depth_pass_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));  



     // clear accumulation FBOs color attachment

     ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f,0.0f)); 


     // clear normalization FBOs color attachment
     ctx.render_context->clear_color_buffer(normalization_pass_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f,0.0f));

      } 




  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  PBRUberShader::predraw(RenderContext const& ctx,
                                               std::string const& file_name,
                                               std::string const& material_name,
                                               scm::math::mat4 const& model_matrix,
                                               scm::math::mat4 const& normal_matrix,
                                               Frustum const& frustum,
                                               std::size_t viewid) const
  {

      auto pbr_ressource     = std::static_pointer_cast<PBRRessource>(GeometryDatabase::instance()->lookup(file_name));
      auto material          = MaterialDatabase::instance()->lookup(material_name);

	   // begin of depth pass (first)
	  {

            if( last_geometry_state_[ctx.id] != pre_draw_state)
            {
	      
              //enable dynamic point size in shaders
	      ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);

	     
	      // bind fbo
	      ctx.render_context->set_frame_buffer(depth_pass_result_fbo_[ctx.id]);



	      gua::math::mat4 const& projection_matrix = frustum.get_projection(); 

	      float   near_plane_value = frustum.get_clip_near();
              float   far_plane_value  = frustum.get_clip_far();

              std::vector<math::vec3> corner_values = frustum.get_corners();
              float top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
              float height_divided_by_top_minus_bottom = (render_window_dims_[ctx.id])[1] / top_minus_bottom;

	      get_program(depth_pass)->set_uniform(ctx, height_divided_by_top_minus_bottom, "height_divided_by_top_minus_bottom");
	      get_program(depth_pass)->set_uniform(ctx, near_plane_value, "near_plane");
	      get_program(depth_pass)->set_uniform(ctx, (far_plane_value - near_plane_value), "far_minus_near_plane");
              
              last_geometry_state_[ctx.id] = pre_draw_state;


           }


              scm::math::vec4f x_unit_vec(1.0f,0.f,0.f,0.f);

              float radius_model_scaling = scm::math::length(model_matrix * x_unit_vec);


              get_program(depth_pass)->set_uniform(ctx, radius_model_scaling, "radius_model_scaling");

              get_program(depth_pass)->set_uniform(ctx, transpose(inverse(frustum.get_view()*model_matrix)), "gua_normal_matrix");
	      get_program(depth_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");


	      if (material && pbr_ressource)
	      {
		get_program(depth_pass)->use(ctx);
		{
		  pbr_ressource->draw(ctx);
		}
		get_program(depth_pass)->unuse(ctx);
	      }


	    }

  }



////////////////////////////////////////////////////////////////////////////////

void PBRUberShader::draw(RenderContext const& ctx,
                             std::string const& file_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& frustum,
                             std::size_t viewid) const
{
    throw std::runtime_error("PBRUberShader::draw(): not implemented");
}






  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void PBRUberShader::postdraw(RenderContext const& ctx,
    std::string const& file_name,
    std::string const& material_name,
    scm::math::mat4 const& model_matrix,
    scm::math::mat4 const& normal_matrix,
    Frustum const& frustum,
    std::size_t viewid) const
  {

  auto pbr_ressource     = std::static_pointer_cast<PBRRessource>(GeometryDatabase::instance()->lookup(file_name));
  auto material          = MaterialDatabase::instance()->lookup(material_name);

  {

      if( last_geometry_state_[ctx.id] != post_draw_state)
      {

        ctx.render_context->reset();
        //enable dynamic point size in shaders
        ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);

        //disable depth test
        ctx.render_context->set_depth_stencil_state(no_depth_test_depth_stencil_state_[ctx.id]);
 
        //set blend state to accumulate
        ctx.render_context->set_blend_state(color_accumulation_state_[ctx.id]);

       // ctx.render_context->clear_color_buffer(accumulation_pass_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 1.0f, 0.0f, 0.0f)); 
        // bind accumulation FBO
        ctx.render_context->set_frame_buffer(accumulation_pass_result_fbo_[ctx.id]);
   
        gua::math::mat4 const& projection_matrix = frustum.get_projection(); 

        //put near_plane_value and height_divided_by_top_minus_bottom as member variables and get these two only 1 per render frame
        float   near_plane_value = frustum.get_clip_near();
        float   far_plane_value  = frustum.get_clip_far();

        std::vector<math::vec3> corner_values = frustum.get_corners();
        float top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
        float height_divided_by_top_minus_bottom = (render_window_dims_[ctx.id])[1] / top_minus_bottom;

        get_program(accumulation_pass)->set_uniform(ctx, height_divided_by_top_minus_bottom, "height_divided_by_top_minus_bottom");
        get_program(accumulation_pass)->set_uniform(ctx, near_plane_value, "near_plane");
        get_program(accumulation_pass)->set_uniform(ctx, (far_plane_value - near_plane_value), "far_minus_near_plane");

        get_program(accumulation_pass)->set_uniform(ctx, math::vec2(render_window_dims_[ctx.id]),"win_dims");


        last_geometry_state_[ctx.id] = post_draw_state;


      }

        scm::math::vec4f x_unit_vec(1.0f,0.f,0.f,0.f);

        float radius_model_scaling = scm::math::length(model_matrix * x_unit_vec);


        get_program(accumulation_pass)->set_uniform(ctx, radius_model_scaling, "radius_model_scaling");

        ctx.render_context->bind_texture(depth_pass_linear_depth_result_[ctx.id], linear_sampler_state_[ctx.id], 0);
        get_program(accumulation_pass)->get_program(ctx)->uniform_sampler("p01_depth_texture", 0);

      
        get_program(accumulation_pass)->set_uniform(ctx, transpose(inverse(frustum.get_view()*model_matrix)), "gua_normal_matrix");
        get_program(accumulation_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");

      if (material && pbr_ressource)
      {
         
        material_id_[ctx.id] = material->get_id();
        get_program(accumulation_pass)->use(ctx);
        {
          
          pbr_ressource->draw(ctx);
        }
        get_program(accumulation_pass)->unuse(ctx);
      }

    }

  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void PBRUberShader::postframe(RenderContext const& ctx) const
  {


          (context_guard_[ctx.id]).reset();

	  {


            {
	    scm::gl::context_all_guard guard(ctx.render_context);


            // bind normalization FBO
            ctx.render_context->set_frame_buffer(normalization_pass_result_fbo_[ctx.id]);

	    get_program(normalization_pass)->use(ctx);
	    {

	      {

		get_program(normalization_pass)->set_uniform(ctx, 1 , "using_default_pbr_material");


                //bind color output for gbuffer
		ctx.render_context->bind_texture(accumulation_pass_color_result_[ctx.id], linear_sampler_state_[ctx.id], 0);
		get_program(normalization_pass)->get_program(ctx)->uniform_sampler("p02_color_texture", 0);
 
		fullscreen_quad_[ctx.id]->draw(ctx.render_context);
	      }
	    }
	    get_program(normalization_pass)->unuse(ctx);

          }
	   // ctx.render_context->reset_framebuffer();


         /////////////////////////////////////////////////////////////////////////////////////////////
         //insert screen space reconstruction pass here

         {

	    scm::gl::context_all_guard guard(ctx.render_context);

	    get_program(reconstruction_pass)->use(ctx);
	    {

	      {

		get_program(reconstruction_pass)->set_uniform(ctx, material_id_[ctx.id], "gua_material_id");


		get_program(reconstruction_pass)->set_uniform(ctx, 1 , "using_default_pbr_material");

                get_program(reconstruction_pass)->set_uniform(ctx, math::vec2(render_window_dims_[ctx.id]),"win_dims");

                //bind logarithmic depth texture for gbuffer
		ctx.render_context->bind_texture(depth_pass_log_depth_result_[ctx.id], linear_sampler_state_[ctx.id], 0);
		get_program(reconstruction_pass)->get_program(ctx)->uniform_sampler("p01_depth_texture", 0);


                //bind color output for gbuffer
		ctx.render_context->bind_texture(normalization_pass_color_result_[ctx.id], linear_sampler_state_[ctx.id], 1);
		get_program(reconstruction_pass)->get_program(ctx)->uniform_sampler("p02_color_texture", 1);
 
		fullscreen_quad_[ctx.id]->draw(ctx.render_context);
	      }
	    }
	    get_program(reconstruction_pass)->unuse(ctx);

	    ctx.render_context->reset_framebuffer();


        }
     }



  }


////////////////////////////////////////////////////////////////////////////////

  std::string const PBRUberShader::default_pbr_material_name() const
  {
    return "gua_pbr";
  }

}

