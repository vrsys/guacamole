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
#include <gua/renderer/Video3DUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/memory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/Window.hpp>
#include <gua/renderer/Video3DRessource.hpp>

#include <gua/databases/MaterialDatabase.hpp>

#include <scm/gl_core/render_device/context_guards.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Video3DUberShader::Video3DUberShader()
  : GeometryUberShader()
{}

////////////////////////////////////////////////////////////////////////////////

void Video3DUberShader::create(std::set<std::string> const& material_names)
{
  UberShader::create(material_names);

  // create depth shader
  std::vector<ShaderProgramStage> warp_pass_stages;
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _warp_pass_vertex_shader()));
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _warp_pass_geometry_shader()));
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _warp_pass_fragment_shader()));

  auto warp_pass_program = std::make_shared<ShaderProgram>();
  warp_pass_program->set_shaders(warp_pass_stages);
  add_program(warp_pass_program);

  // create final shader
  std::vector<ShaderProgramStage> blend_pass_stages;
  blend_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, _blend_pass_vertex_shader()));
  blend_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER, _blend_pass_fragment_shader()));

  auto blend_pass_program = std::make_shared<ShaderProgram>();
  blend_pass_program->set_shaders(blend_pass_stages);
  add_program(blend_pass_program);
}

////////////////////////////////////////////////////////////////////////////////

std::string const Video3DUberShader::_warp_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_warp_pass_vert)
    );

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const Video3DUberShader::_warp_pass_geometry_shader() const
{
  std::string geometry_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_warp_pass_geom)
  );

  return geometry_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const Video3DUberShader::_warp_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_warp_pass_frag)
    );

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const Video3DUberShader::_blend_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_blend_pass_vert)
  );

  // material specific uniforms
  string_utils::replace(vertex_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // output
  string_utils::replace(vertex_shader, "@output_definition",
    vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true));

  // print material specific methods
  string_utils::replace(vertex_shader, "@material_methods",
    UberShader::print_material_methods(*vshader_factory_));

  // print main switch(es)
  string_utils::replace(vertex_shader, "@material_switch",
    UberShader::print_material_switch(*vshader_factory_));

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const Video3DUberShader::_blend_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_blend_pass_frag)
    );

  std::string apply_video_color = fshader_factory_->get_output_mapping().get_output_string(default_video_material_name(), "gua_video_output_color");
  apply_video_color += " = output_color;\n";

  std::string apply_video_normal = fshader_factory_->get_output_mapping().get_output_string(default_video_material_name(), "gua_normal");
  apply_video_normal += " = output_normal;\n";

  string_utils::replace(fragment_shader, "@apply_video3d_color", apply_video_color);
  string_utils::replace(fragment_shader, "@apply_video3d_normal", apply_video_normal);

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

bool Video3DUberShader::upload_to (RenderContext const& context) const
{
  bool upload_succeeded = UberShader::upload_to(context);

  if (warp_color_result_) {
    return upload_succeeded;
  }
  else {
      // continue instantiation below
  }

  assert(context.render_window->config.get_stereo_mode() == StereoMode::MONO ||
	 ((context.render_window->config.get_left_resolution()[0] == context.render_window->config.get_right_resolution()[0]) &&
	  (context.render_window->config.get_left_resolution()[1] == context.render_window->config.get_right_resolution()[1])));


  // initialize Texture Arrays (kinect depths & colors)

  warp_color_result_ = context.render_device->create_texture_2d(
								context.render_window->config.get_left_resolution(),
								scm::gl::FORMAT_RGBA_32F,
								1,
								MAX_NUM_KINECTS,
								1
								);
  
  warp_depth_result_ = context.render_device->create_texture_2d(
								context.render_window->config.get_left_resolution(),
								scm::gl::FORMAT_D32F,
								1,
								MAX_NUM_KINECTS,
								1
								);

  fullscreen_quad_.reset(new scm::gl::quad_geometry(context.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));


  warp_result_fbo_ = context.render_device->create_frame_buffer();
  

  linear_sampler_state_ = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
  

  nearest_sampler_state_ = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
 

  depth_stencil_state_warp_pass_ = context.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);


  depth_stencil_state_blend_pass_ = context.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
 
  no_bfc_rasterizer_state_ = context.render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE);


  return upload_succeeded;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ GeometryUberShader::stage_mask const Video3DUberShader::get_stage_mask() const
{
  return GeometryUberShader::DRAW_STAGE | GeometryUberShader::POST_FRAME_STAGE;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void  Video3DUberShader::preframe(RenderContext const& context) const
{
  throw std::runtime_error("not implemented");
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void  Video3DUberShader::predraw(RenderContext const& ctx,
  std::string const& filename,
  std::string const& material_name,
  scm::math::mat4 const& model_matrix,
  scm::math::mat4 const& normal_matrix,
  Frustum const& /*frustum*/,
  std::size_t viewid) const
{
  throw std::runtime_error("not implemented");
}


////////////////////////////////////////////////////////////////////////////////

void Video3DUberShader::draw(RenderContext const& ctx,
                             std::string const& ksfile_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& /*frustum*/,
                             std::size_t viewid) const
{

  if (!GeometryDatabase::instance()->is_supported(ksfile_name) ||
      !MaterialDatabase::instance()->is_supported(material_name)) {
    gua::Logger::LOG_WARNING << "Video3DUberShader::draw(): No such video or material." << ksfile_name << ", " << material_name << std::endl;
    return;
  }

  auto video3d_ressource = std::static_pointer_cast<Video3DRessource>(GeometryDatabase::instance()->lookup(ksfile_name));
  auto material          = MaterialDatabase::instance()->lookup(material_name);

  if (!video3d_ressource || !material) {
    gua::Logger::LOG_WARNING << "Video3DUberShader::draw(): Invalid video or material." << std::endl;
    return;
  }

  // update stream data
  video3d_ressource->update_buffers(ctx);

  // make sure ressources are on the GPU
  upload_to(ctx);
  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    ctx.render_context->set_rasterizer_state(no_bfc_rasterizer_state_);
    ctx.render_context->set_depth_stencil_state(depth_stencil_state_warp_pass_);

    // set uniforms
    ctx.render_context->bind_texture(video3d_ressource->depth_array(ctx), nearest_sampler_state_, 0);
    get_program(warp_pass)->get_program(ctx)->uniform_sampler("depth_video3d_texture", 0);


    get_program(warp_pass)->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
    get_program(warp_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");
    get_program(warp_pass)->set_uniform(ctx, int(1), "bbxclip");

    auto bbox(video3d_ressource->get_bounding_box());
    get_program(warp_pass)->set_uniform(ctx, bbox.min, "bbx_min");
    get_program(warp_pass)->set_uniform(ctx, bbox.max, "bbx_max");

    // pre passes
    for (unsigned layer = 0; layer != video3d_ressource->number_of_cameras(); ++layer)
    {
      // configure fbo
      warp_result_fbo_->clear_attachments();
      warp_result_fbo_->attach_depth_stencil_buffer(warp_depth_result_, 0, layer);
      warp_result_fbo_->attach_color_buffer(0, warp_color_result_, 0, layer);

      // bind and clear fbo
      ctx.render_context->set_frame_buffer(warp_result_fbo_);
      ctx.render_context->clear_depth_stencil_buffer(warp_result_fbo_);
      ctx.render_context->clear_color_buffer(warp_result_fbo_, 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
      ctx.render_context->set_viewport(scm::gl::viewport(scm::math::vec2ui(0,0), warp_color_result_->dimensions()));

      // set uniforms
      get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).getTexSizeInvD(), "tex_size_inv");
      get_program(warp_pass)->set_uniform(ctx, int(layer), "layer");
      

      if (material && video3d_ressource)
      {
        get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).getImageDToEyeD(), "image_d_to_eye_d");
        get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeDToWorld(), "eye_d_to_world");
        get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeDToEyeRGB(), "eye_d_to_eye_rgb");
        get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeRGBToImageRGB(), "eye_rgb_to_image_rgb");

	      ctx.render_context->bind_texture(video3d_ressource->cv_xyz(ctx,layer), linear_sampler_state_, 1);
	      get_program(warp_pass)->get_program(ctx)->uniform_sampler("cv_xyz", 1);

	      ctx.render_context->bind_texture(video3d_ressource->cv_uv(ctx,layer), linear_sampler_state_, 2);
	      get_program(warp_pass)->get_program(ctx)->uniform_sampler("cv_uv", 2);

	      get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).cv_min_d, "cv_min_d");
	      get_program(warp_pass)->set_uniform(ctx, video3d_ressource->calibration_file(layer).cv_max_d, "cv_max_d");

        get_program(warp_pass)->use(ctx);
        {
          video3d_ressource->draw(ctx);
        }
        get_program(warp_pass)->unuse(ctx);
      }

      ctx.render_context->reset_framebuffer();
    }
  }

  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    ctx.render_context->set_depth_stencil_state(depth_stencil_state_warp_pass_);

    // second pass
    get_program(blend_pass)->use(ctx);
    {
      if (material && video3d_ressource)
      {
        set_uniform(ctx, material->get_id(), "gua_material_id");
        set_uniform(ctx, normal_matrix, "gua_normal_matrix");
        set_uniform(ctx, model_matrix, "gua_model_matrix");

        // needs to be multiplied with scene scaling
        set_uniform(ctx, 0.075f, "epsilon");
        set_uniform(ctx, int(video3d_ressource->number_of_cameras()), "numlayers");
        get_program(blend_pass)->set_uniform(ctx, int(material_name == default_video_material_name()), "using_default_video_material");
        get_program(blend_pass)->set_uniform(ctx, int(video3d_ressource->do_overwrite_normal()), "overwrite_normal");
        get_program(blend_pass)->set_uniform(ctx, video3d_ressource->get_overwrite_normal(), "o_normal");

        ctx.render_context->bind_texture(warp_color_result_, nearest_sampler_state_, 0);
        get_program(blend_pass)->get_program(ctx)->uniform_sampler("quality_texture", 0);

        ctx.render_context->bind_texture(warp_depth_result_, nearest_sampler_state_, 1);
        get_program(blend_pass)->get_program(ctx)->uniform_sampler("depth_texture", 1);

        ctx.render_context->bind_texture(video3d_ressource->color_array(ctx), linear_sampler_state_, 2);
        get_program(blend_pass)->get_program(ctx)->uniform_sampler("video_color_texture", 2);

        fullscreen_quad_->draw(ctx.render_context);
      }
    }
    get_program(blend_pass)->unuse(ctx);
  }
}


////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void Video3DUberShader::postdraw(RenderContext const& ctx,
  std::string const& filename,
  std::string const& material_name,
  scm::math::mat4 const& model_matrix,
  scm::math::mat4 const& normal_matrix,
  Frustum const& /*frustum*/,
  std::size_t viewid) const
{
  throw std::runtime_error("not implemented");
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void Video3DUberShader::postframe(RenderContext const& context) const
{
  // update could/should be triggered from here -> use context to keep frame-coherence
}

////////////////////////////////////////////////////////////////////////////////

/*static*/ std::string const Video3DUberShader::default_video_material_name() {
  return "gua_video3d";
}

////////////////////////////////////////////////////////////////////////////////

/*static*/ void Video3DUberShader::initialize_video_material() {
  if (!MaterialDatabase::instance()->is_supported(default_video_material_name()))
  {
    create_resource_material(default_video_material_name(),
      Resources::materials_gua_video3d_gsd,
      Resources::materials_gua_video3d_gmd);
  }
}

}

