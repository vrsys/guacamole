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
#include <gua/renderer/PLODUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/Window.hpp>
#include <gua/renderer/PLODRessource.hpp>
#include <gua/renderer/View.hpp>
#include <gua/databases.hpp>

#include <pbr/ren/camera.h>

#include <pbr/ren/policy.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/cut_database.h>
#include <pbr/ren/controller.h>

#include <pbr/ren/raw_point_cloud.h>

#include <gua/databases/MaterialDatabase.hpp>

#include <gua/utils/Logger.hpp>

#include <boost/assign/list_of.hpp>

namespace gua {


////////////////////////////////////////////////////////////////////////////////

PLODUberShader::PLODUberShader()
  : GeometryUberShader(), near_plane_value_(0.0f), height_divided_by_top_minus_bottom_(0.0f)
{}


  ////////////////////////////////////////////////////////////////////////////////

  void PLODUberShader::create(std::set<std::string> const& material_names)
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

PLODUberShader::~PLODUberShader()
{
    std::cout << "memory cleanup...(1)" << std::endl;
    delete pbr::ren::Controller::GetInstance();
    std::cout << "deleted controller" << std::endl;
    delete pbr::ren::ModelDatabase::GetInstance();
    std::cout << "deleted model database" << std::endl;
    delete pbr::ren::Policy::GetInstance();
    std::cout << "deleted memory query" << std::endl;
    delete pbr::ren::CutDatabase::GetInstance();
    std::cout << "deleted cut database" << std::endl;
}


////////////////////////////////////////////////////////////////////////////////

std::string const PLODUberShader::depth_pass_vertex_shader() const
{

  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p01_depth_vert)
    );

  return vertex_shader;
}



////////////////////////////////////////////////////////////////////////////////

std::string const PLODUberShader::depth_pass_fragment_shader() const
{

  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p01_depth_frag)
    );

  return fragment_shader;
}



////////////////////////////////////////////////////////////////////////////////



std::string const PLODUberShader::accumulation_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p02_accumulation_vert)
    );
  return vertex_shader;
}



////////////////////////////////////////////////////////////////////////////////

std::string const PLODUberShader::accumulation_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p02_accumulation_frag)
    );

  return fragment_shader;
}


////////////////////////////////////////////////////////////////////////////////



std::string const PLODUberShader::normalization_pass_vertex_shader() const
{
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_vert)
  );

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string const PLODUberShader::normalization_pass_fragment_shader() const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_frag)
    );

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////



std::string const PLODUberShader::reconstruction_pass_vertex_shader() const
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

std::string const PLODUberShader::reconstruction_pass_fragment_shader() const
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

bool PLODUberShader::upload_to (RenderContext const& context) const
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
  
 
  if (context.id >= temp_buffer_A_.size())
  {
  
    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

    size_t size_of_node_in_bytes = database->size_of_surfel()*database->surfels_per_node();
  
    int32_t upload_budget_in_nodes =
        (policy->upload_budget_in_mb()*1024*1024) / size_of_node_in_bytes;


    unsigned long long sizeOfNode = database->surfels_per_node() *database->size_of_surfel();
    unsigned long long sizeOfTempBuffers =  upload_budget_in_nodes * sizeOfNode;
    
    temp_buffer_A_.resize(context.id + 1);
    temp_buffer_A_[context.id] = context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, sizeOfTempBuffers, NULL); 
  }
  

  if (context.id >= temp_buffer_B_.size())
  {
    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

    size_t size_of_node_in_bytes = database->size_of_surfel()*database->surfels_per_node();
  
    int32_t upload_budget_in_nodes =
        (policy->upload_budget_in_mb()*1024*1024) / size_of_node_in_bytes;


    unsigned long long sizeOfNode = database->surfels_per_node() *database->size_of_surfel();
    unsigned long long sizeOfTempBuffers =  upload_budget_in_nodes * sizeOfNode;
    
    temp_buffer_B_.resize(context.id + 1);
    temp_buffer_B_[context.id] = context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, sizeOfTempBuffers, NULL);   
  }

  if (context.id >= render_buffer_.size())
  {
    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

    size_t size_of_node_in_bytes = database->size_of_surfel()*database->surfels_per_node();

    int32_t render_budget_in_nodes =
        (policy->render_budget_in_mb()*1024*1024) / size_of_node_in_bytes;
        
    unsigned long long sizeOfNode = database->surfels_per_node() *database->size_of_surfel();  
    unsigned long long sizeOfRenderBuffer = render_budget_in_nodes * sizeOfNode;
    

    
    render_buffer_.resize(context.id + 1);
    render_buffer_[context.id] = context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER, scm::gl::USAGE_DYNAMIC_COPY, sizeOfRenderBuffer);
  }


  if (context.id >= vertex_array_.size())
  {
  

     vertex_array_.resize(context.id + 1);
     
     vertex_array_[context.id] = context.render_device->create_vertex_array(
         scm::gl::vertex_format(
             0, 0, scm::gl::TYPE_VEC3F, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
             0, 1, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
             0, 2, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
             0, 3, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
             0, 4, scm::gl::TYPE_UBYTE, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
             0, 5, scm::gl::TYPE_FLOAT, sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
             0, 6, scm::gl::TYPE_VEC3F, sizeof(pbr::ren::RawPointCloud::SerializedSurfel)),
          boost::assign::list_of(render_buffer_[context.id]));

  }
  
  if (context.id >= temp_buffer_A_is_mapped_.size())
  {
    temp_buffer_A_is_mapped_.resize(context.id + 1);
    temp_buffer_A_is_mapped_[context.id] = false;
  }
  
  if (context.id >= temp_buffer_B_is_mapped_.size())
  {
    temp_buffer_B_is_mapped_.resize(context.id + 1);
    temp_buffer_B_is_mapped_[context.id] = false;
  }
  

  if (context.id >= mapped_temp_buffer_A_.size())
  {
    mapped_temp_buffer_A_.resize(context.id + 1);
    mapped_temp_buffer_A_[context.id] = nullptr;
  }
  
  if (context.id >= mapped_temp_buffer_B_.size())
  {
    mapped_temp_buffer_B_.resize(context.id + 1);
    mapped_temp_buffer_B_[context.id] = nullptr;
  }
  
  if (context.id >= previous_framecount_.size())
  {
    previous_framecount_.resize(context.id + 1);
    previous_framecount_[context.id] = -1;
  }
  
  if(context.id >= frustum_culling_results_.size())
  {
    frustum_culling_results_.resize(context.id + 1);
    frustum_culling_results_[context.id].clear();
  }

  return upload_succeeded;
}






  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ GeometryUberShader::stage_mask const PLODUberShader::get_stage_mask() const
  {

    return GeometryUberShader::PRE_FRAME_STAGE | GeometryUberShader::PRE_DRAW_STAGE | GeometryUberShader::POST_DRAW_STAGE | GeometryUberShader::POST_FRAME_STAGE;
  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void  PLODUberShader::preframe(RenderContext const& ctx) const
  {
    upload_to(ctx);

    if (previous_framecount_[ctx.id] != ctx.framecount)
    {

      previous_framecount_[ctx.id] = ctx.framecount;
    
      pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
      pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
      pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
    
      pbr::context_t context_id = controller->DeduceContextId(ctx.id);

      //swap cut database
      cuts->AcceptFront(context_id);
      

      if (temp_buffer_A_is_mapped_[ctx.id])
      {
          //run cut update
          controller->DispatchCutUpdate(context_id);
      }
      else
      {
          //startup
          
          controller->StoreTemporaryBuffers(
              context_id,
              GetMappedTempBufferPtr(ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A),
              GetMappedTempBufferPtr(ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B));
              
         database->set_window_width(render_window_dims_[ctx.id][0]);
         database->set_window_height(render_window_dims_[ctx.id][1]);
      }
      
      if (cuts->IsFrontModified(context_id))
      {
          //upload data to gpu
          
          UnmapTempBufferPtr(ctx, cuts->GetBuffer(context_id));

          CopyTempToMainMemory(ctx, cuts->GetBuffer(context_id));

          cuts->SignalModificationComplete(context_id);

          controller->StoreTemporaryBuffers(
              context_id,
              GetMappedTempBufferPtr(ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A),
              GetMappedTempBufferPtr(ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B));

      }
      
    }




      //scm::gl::context_all_guard guard(ctx.render_context);


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

  /*virtual*/ void  PLODUberShader::predraw(RenderContext const& ctx,
                                               std::string const& file_name,
                                               std::string const& material_name,
                                               scm::math::mat4 const& model_matrix,
                                               scm::math::mat4 const& normal_matrix,
                                               Frustum const& frustum,
                                               View const& view) const
  {
      std::vector<math::vec3> corner_values = frustum.get_corners();
      float top_minus_bottom = scm::math::length((corner_values[2]) - (corner_values[0]));
      float height_divided_by_top_minus_bottom = (render_window_dims_[ctx.id])[1] / top_minus_bottom;
      float near_plane_value = frustum.get_clip_near();
      float far_plane_value  = frustum.get_clip_far();

      if( last_geometry_state_[ctx.id] != pre_draw_state)
      {
	      
        //enable dynamic point size in shaders
	ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);

	// bind fbo
	ctx.render_context->set_frame_buffer(depth_pass_result_fbo_[ctx.id]);

	get_program(depth_pass)->set_uniform(ctx, height_divided_by_top_minus_bottom, "height_divided_by_top_minus_bottom");
	get_program(depth_pass)->set_uniform(ctx, near_plane_value, "near_plane");
	get_program(depth_pass)->set_uniform(ctx, (far_plane_value - near_plane_value), "far_minus_near_plane");
              
        last_geometry_state_[ctx.id] = pre_draw_state;

      }

      pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
      pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
      
      pbr::context_t context_id = controller->DeduceContextId(ctx.id);
      pbr::view_t view_id = controller->DeduceViewId(context_id, view.id);
      pbr::model_t model_id = controller->DeduceModelId(file_name);
      
      //send camera and model_matrix to cut update
      pbr::ren::Camera cut_update_cam(view_id, near_plane_value, frustum.get_view(), frustum.get_projection() );

      cuts->SendCamera(context_id, view_id, cut_update_cam);
      cuts->SendHeightDividedByTopMinusBottom(context_id, view_id, height_divided_by_top_minus_bottom);
      cuts->SendTransform(context_id, model_id, model_matrix);
      
   
      pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
      std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();

      //calculate frustum culling results

      pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

      pbr::ren::KdnTree const*  kdn_tree = database->GetModel(model_id)->kdn_tree();

      scm::gl::frustum culling_frustum = cut_update_cam.GetFrustumByModel(model_matrix);

      std::vector<scm::gl::boxf> const& model_bounding_boxes = kdn_tree->bounding_boxes();

      unsigned int node_counter = 0;

      frustum_culling_results_[ctx.id].clear();
      frustum_culling_results_[ctx.id].resize(model_bounding_boxes.size());

      for(std::vector<pbr::ren::Cut::NodeSlotAggregate>::const_iterator k = node_list.begin(); k != node_list.end(); ++k, ++node_counter)
      {
          (frustum_culling_results_[ctx.id])[node_counter] = culling_frustum.classify(model_bounding_boxes[k->node_id_]);
      }
            
      auto plod_ressource     = std::static_pointer_cast<PLODRessource>(GeometryDatabase::instance()->lookup(file_name));
      auto material          = MaterialDatabase::instance()->lookup(material_name);
   
      // begin of depth pass (first)
      {
          scm::math::vec4f x_unit_vec(1.0,0.0f,0.f,0.f);
          float radius_model_scaling = scm::math::length(model_matrix * x_unit_vec);
          get_program(depth_pass)->set_uniform(ctx, radius_model_scaling, "radius_model_scaling");
	  get_program(depth_pass)->set_uniform(ctx, transpose(inverse(frustum.get_view()*model_matrix)), "gua_normal_matrix");
	  get_program(depth_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");

	  if (material && plod_ressource)
	  {
              get_program(depth_pass)->use(ctx);
              plod_ressource->draw(ctx, context_id, view_id, model_id, vertex_array_[ctx.id], frustum_culling_results_[ctx.id] );
              get_program(depth_pass)->unuse(ctx);
	  }
      }

  }



////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::draw(RenderContext const& ctx,
                             std::string const& file_name,
                             std::string const& material_name,
                             scm::math::mat4 const& model_matrix,
                             scm::math::mat4 const& normal_matrix,
                             Frustum const& frustum,
                             View const& view) const
{
    throw std::runtime_error("PLODUberShader::draw(): not implemented");
}

  ////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void PLODUberShader::postdraw(RenderContext const& ctx,
  std::string const& file_name,
  std::string const& material_name,
  scm::math::mat4 const& model_matrix,
  scm::math::mat4 const& normal_matrix,
  Frustum const& frustum,
  View const& view) const {

  auto plod_ressource     = std::static_pointer_cast<PLODRessource>(GeometryDatabase::instance()->lookup(file_name));
  auto material          = MaterialDatabase::instance()->lookup(material_name);

  {


      float   near_plane_value = frustum.get_clip_near();
      float   far_plane_value  = frustum.get_clip_far();

      if( last_geometry_state_[ctx.id] != post_draw_state)
      {

        ctx.render_context->reset();
        //enable dynamic point size in shaders
        ctx.render_context->set_rasterizer_state(change_point_size_in_shader_state_[ctx.id]);

        //disable depth test
        ctx.render_context->set_depth_stencil_state(no_depth_test_depth_stencil_state_[ctx.id]);
 
        //set blend state to accumulate
        ctx.render_context->set_blend_state(color_accumulation_state_[ctx.id]);

        // bind accumulation FBO
        ctx.render_context->set_frame_buffer(accumulation_pass_result_fbo_[ctx.id]);

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

      if (material && plod_ressource)
      {
         
        pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
        pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
        pbr::context_t context_id = controller->DeduceContextId(ctx.id);
        pbr::model_t model_id = controller->DeduceModelId(file_name);
        pbr::view_t view_id = controller->DeduceViewId(context_id, view.id);
         
        material_id_[ctx.id] = material->get_id();


      pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
      pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
      std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();

      //calculate frustum culling results
      pbr::ren::Camera cut_update_cam(view_id, near_plane_value, frustum.get_view(), frustum.get_projection() );

      pbr::ren::KdnTree const*  kdn_tree = database->GetModel(model_id)->kdn_tree();

      scm::gl::frustum culling_frustum = cut_update_cam.GetFrustumByModel(model_matrix);

      std::vector<scm::gl::boxf> const& model_bounding_boxes = kdn_tree->bounding_boxes();

      unsigned int node_counter = 0;

      frustum_culling_results_[ctx.id].clear();
      frustum_culling_results_[ctx.id].resize(model_bounding_boxes.size());

      for(std::vector<pbr::ren::Cut::NodeSlotAggregate>::const_iterator k = node_list.begin(); k != node_list.end(); ++k, ++node_counter)
      {
          (frustum_culling_results_[ctx.id])[node_counter] = culling_frustum.classify(model_bounding_boxes[k->node_id_]);
      }


        get_program(accumulation_pass)->use(ctx);
        {
          
          plod_ressource->draw(ctx, context_id, view_id, model_id, vertex_array_[ctx.id], frustum_culling_results_[ctx.id]);
        }
        get_program(accumulation_pass)->unuse(ctx);
      }

    }

  }

  ////////////////////////////////////////////////////////////////////////////////

  /*virtual*/ void PLODUberShader::postframe(RenderContext const& ctx) const
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

char* PLODUberShader::GetMappedTempBufferPtr(RenderContext const& ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const
{
    switch (buffer)
    {
        case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A:
            if (!temp_buffer_A_is_mapped_[ctx.id])
            {
                mapped_temp_buffer_A_[ctx.id] = (char*)ctx.render_context->map_buffer(temp_buffer_A_[ctx.id], scm::gl::ACCESS_READ_WRITE );
                temp_buffer_A_is_mapped_[ctx.id] = true;
            }
            return mapped_temp_buffer_A_[ctx.id];
            break;

        case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B:
            if (!temp_buffer_B_is_mapped_[ctx.id])
            {
                mapped_temp_buffer_B_[ctx.id] = (char*)ctx.render_context->map_buffer(temp_buffer_B_[ctx.id], scm::gl::ACCESS_READ_WRITE );
                temp_buffer_B_is_mapped_[ctx.id] = true;
            }
            return mapped_temp_buffer_B_[ctx.id];
            break;

        default: break;
    }

    std::cout << "Failed to map temporary buffer.\n";
    assert(false);

    return nullptr;
}
////////////////////////////////////////////////////////////////////////////////
void PLODUberShader::UnmapTempBufferPtr(RenderContext const& ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const
{
    switch (buffer)
    {
        case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A:
            if (temp_buffer_A_is_mapped_[ctx.id])
            {
                ctx.render_context->unmap_buffer(temp_buffer_A_[ctx.id]);
                temp_buffer_A_is_mapped_[ctx.id] = false;
            }
            break;

        case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B:
            if (temp_buffer_B_is_mapped_[ctx.id])
            {
                ctx.render_context->unmap_buffer(temp_buffer_B_[ctx.id]);
                temp_buffer_B_is_mapped_[ctx.id] = false;
            }
            break;

        default: break;
    }

}
////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::CopyTempToMainMemory(RenderContext const& ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer const&  buffer) const
{

    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

    size_t size_of_node_in_bytes = database->surfels_per_node() * database->size_of_surfel();


    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();

    //uploaded_nodes_ = 0;

    pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();

    pbr::context_t context_id = controller->DeduceContextId(ctx.id);

    switch (buffer)
    {
        case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A:
        {
            if (temp_buffer_A_is_mapped_[ctx.id])
            {
                std::cout << "Failed to transfer nodes into main memory.\nTemp Storage A was still mapped.";
                assert(false);
            }
            std::vector<pbr::ren::CutDatabaseRecord::SlotUpdateDescr>& transfer_descr_list = cuts->GetUpdatedSet(context_id);
            
            if (!transfer_descr_list.empty())
            {
                for (const auto& transfer_desc : transfer_descr_list)
                {
                    size_t offset_in_temp_VBO = transfer_desc.src_ * size_of_node_in_bytes;
                    size_t offset_in_render_VBO = transfer_desc.dst_ * size_of_node_in_bytes;
                    ctx.render_context->copy_buffer_data(render_buffer_[ctx.id],temp_buffer_A_[ctx.id], offset_in_render_VBO, offset_in_temp_VBO, size_of_node_in_bytes);
                }
            }
            break;
        }

        case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B:
        {
            if (temp_buffer_B_is_mapped_[ctx.id])
            {
                std::cout << "Failed to transfer nodes into main memory.\nTemp Storage B was still mapped.";
                assert(false);
            }
            std::vector<pbr::ren::CutDatabaseRecord::SlotUpdateDescr>& transfer_descr_list = cuts->GetUpdatedSet(context_id);
            if (!transfer_descr_list.empty())
            {
                for (const auto& transfer_desc : transfer_descr_list)
                {
                    size_t offset_in_temp_VBO = transfer_desc.src_ * size_of_node_in_bytes;
                    size_t offset_in_render_VBO = transfer_desc.dst_ * size_of_node_in_bytes;
                    ctx.render_context->copy_buffer_data(render_buffer_[ctx.id],temp_buffer_B_[ctx.id], offset_in_render_VBO, offset_in_temp_VBO, size_of_node_in_bytes);
                }
            }
            
            break;
        }

        default: break;

    }


}

////////////////////////////////////////////////////////////////////////////////

  std::string const PLODUberShader::default_plod_material_name() const
  {
    return "gua_pbr";
  }

}

