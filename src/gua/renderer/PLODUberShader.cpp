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

#if 0

// class header
#include <gua/renderer/PLODUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/guacamole.hpp>
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/PLODRessource.hpp>
#include <gua/renderer/View.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <pbr/ren/camera.h>
#include <pbr/ren/policy.h>
#include <pbr/ren/model_database.h>
#include <pbr/ren/cut_database.h>
#include <pbr/ren/controller.h>
#include <pbr/ren/raw_point_cloud.h>
#include <boost/assign/list_of.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::create(std::set<std::string> const& material_names) {

  UberShader::create(material_names);

  // create depth pass shader
  std::vector<ShaderProgramStage> depth_pass_stages;

  depth_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER,
                                                 depth_pass_vertex_shader()));
  depth_pass_stages.push_back(ShaderProgramStage(scm::gl::STAGE_FRAGMENT_SHADER,
                                                 depth_pass_fragment_shader()));

  auto depth_pass_program = std::make_shared<ShaderProgram>();
  depth_pass_program->set_shaders(depth_pass_stages);
  add_program(depth_pass_program);

  // create accumulation pass shader
  std::vector<ShaderProgramStage> accumulation_pass_stages;

  accumulation_pass_stages.push_back(ShaderProgramStage(
      scm::gl::STAGE_VERTEX_SHADER, accumulation_pass_vertex_shader()));
  accumulation_pass_stages.push_back(ShaderProgramStage(
      scm::gl::STAGE_FRAGMENT_SHADER, accumulation_pass_fragment_shader()));

  auto accumulation_pass_program = std::make_shared<ShaderProgram>();
  accumulation_pass_program->set_shaders(accumulation_pass_stages);
  add_program(accumulation_pass_program);

  // create normalization pass shader
  std::vector<ShaderProgramStage> normalization_pass_stages;

  normalization_pass_stages.push_back(ShaderProgramStage(
      scm::gl::STAGE_VERTEX_SHADER, normalization_pass_vertex_shader()));
  normalization_pass_stages.push_back(ShaderProgramStage(
      scm::gl::STAGE_FRAGMENT_SHADER, normalization_pass_fragment_shader()));

  auto normalization_pass_program = std::make_shared<ShaderProgram>();
  normalization_pass_program->set_shaders(normalization_pass_stages);
  add_program(normalization_pass_program);

  // create reconstruction pass shader
  std::vector<ShaderProgramStage> reconstruction_pass_stages;

  reconstruction_pass_stages.push_back(ShaderProgramStage(
      scm::gl::STAGE_VERTEX_SHADER, reconstruction_pass_vertex_shader()));
  reconstruction_pass_stages.push_back(ShaderProgramStage(
      scm::gl::STAGE_FRAGMENT_SHADER, reconstruction_pass_fragment_shader()));

  auto reconstruction_pass_program = std::make_shared<ShaderProgram>();
  reconstruction_pass_program->set_shaders(reconstruction_pass_stages);
  add_program(reconstruction_pass_program);
}

////////////////////////////////////////////////////////////////////////////////

PLODUberShader::~PLODUberShader() {

  Logger::LOG_DEBUG << "memory cleanup...(1)" << std::endl;
  delete pbr::ren::Controller::GetInstance();

  Logger::LOG_DEBUG << "deleted controller" << std::endl;
  delete pbr::ren::ModelDatabase::GetInstance();

  Logger::LOG_DEBUG << "deleted model database" << std::endl;
  delete pbr::ren::Policy::GetInstance();

  Logger::LOG_DEBUG << "deleted memory query" << std::endl;
  delete pbr::ren::CutDatabase::GetInstance();

  Logger::LOG_DEBUG << "deleted cut database" << std::endl;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::depth_pass_vertex_shader() const {

  std::string vertex_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p01_depth_vert));

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::depth_pass_fragment_shader() const {

  std::string fragment_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p01_depth_frag));

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::accumulation_pass_vertex_shader() const {

  std::string vertex_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p02_accumulation_vert));
  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::accumulation_pass_fragment_shader() const {

  std::string fragment_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p02_accumulation_frag));

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::normalization_pass_vertex_shader() const {

  std::string vertex_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_vert));

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////


std::string PLODUberShader::normalization_pass_fragment_shader() const {
  std::string fragment_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p03_normalization_frag));

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::reconstruction_pass_vertex_shader() const {

  std::string vertex_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p04_reconstruction_vert));

  // material specific uniforms

  string_utils::replace(vertex_shader,
                        "@uniform_definition",
                        get_uniform_mapping()->get_uniform_definition());

  // output

  string_utils::replace(
      vertex_shader,
      "@output_definition",
      vshader_factory_->get_output_mapping().get_gbuffer_output_definition(
          false, true));

  return vertex_shader;
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::reconstruction_pass_fragment_shader() const {

  std::string fragment_shader(Resources::lookup_shader(
      Resources::shaders_uber_shaders_gbuffer_pbr_p04_reconstruction_frag));


  std::string apply_pbr_color = fshader_factory_->get_output_mapping()
      .get_output_string("gua_pbr", "gua_pbr_output_color");
  apply_pbr_color += " = output_color;\n";


  std::string apply_pbr_normal = fshader_factory_->get_output_mapping()
      .get_output_string("gua_pbr", "gua_normal");

  apply_pbr_normal += " = output_normal;\n";

  string_utils::replace(fragment_shader, "@apply_pbr_color", apply_pbr_color);
  string_utils::replace(fragment_shader, "@apply_pbr_normal", apply_pbr_normal);

  // input from vertex shader
  string_utils::replace(
      fragment_shader,
      "@input_definition",
      vshader_factory_->get_output_mapping().get_gbuffer_output_definition(
          true, true));

  // material specific uniforms
  string_utils::replace(fragment_shader,
                        "@uniform_definition",
                        get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(
      fragment_shader,
      "@output_definition",
      get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::update_textures(RenderContext const& context) const {

  assert(context.render_window->config.get_stereo_mode() == StereoMode::MONO 
         || left_resolution_ == right_resolution_);

  render_window_dims_ = left_resolution_;

  //initialize attachments for depth pass
  depth_pass_linear_depth_result_ = context.render_device
      ->create_texture_2d(render_window_dims_,
                          scm::gl::FORMAT_D32F,
                          1, 1, 1);

  depth_pass_log_depth_result_ = context.render_device
      ->create_texture_2d(render_window_dims_,
                          scm::gl::FORMAT_R_32F,
                          1, 1, 1);

  //initialize attachments for accumulation pass
  accumulation_pass_color_result_ = context.render_device
      ->create_texture_2d(render_window_dims_,
                          scm::gl::FORMAT_RGBA_32F,
                          1, 1, 1);

  accumulation_pass_normal_result_ = context.render_device
      ->create_texture_2d(render_window_dims_,
                          scm::gl::FORMAT_RGBA_32F,
                          1, 1, 1);

  //initialize attachment for normalization pass
  normalization_pass_color_result_ = context.render_device
      ->create_texture_2d(render_window_dims_,
                          scm::gl::FORMAT_RGB_8,
                          1, 1, 1);

  normalization_pass_normal_result_ = context.render_device
      ->create_texture_2d(render_window_dims_,
                          scm::gl::FORMAT_RGB_32F,
                          1, 1, 1);

  //configure depth FBO
  depth_pass_result_fbo_->clear_attachments();
  depth_pass_result_fbo_->attach_depth_stencil_buffer(
      depth_pass_linear_depth_result_);
  depth_pass_result_fbo_->attach_color_buffer(0,
                                              depth_pass_log_depth_result_);

  //configure accumulation FBO
  accumulation_pass_result_fbo_->clear_attachments();
  accumulation_pass_result_fbo_->attach_color_buffer(
      0, accumulation_pass_color_result_);
  accumulation_pass_result_fbo_->attach_color_buffer(
      1, accumulation_pass_normal_result_);


  //configure normalization FBO
  normalization_pass_result_fbo_->clear_attachments();
  normalization_pass_result_fbo_->attach_color_buffer(
      0, normalization_pass_color_result_);
  normalization_pass_result_fbo_->attach_color_buffer(
      1, normalization_pass_normal_result_);
}

////////////////////////////////////////////////////////////////////////////////

bool PLODUberShader::upload_to(RenderContext const& context) const {

  if (already_uploaded_) {
    if (render_window_dims_ != left_resolution_)
      update_textures(context);
    return false;
  } else {
    already_uploaded_ = true;
  }

  bool upload_succeeded = UberShader::upload_to(context);

  fullscreen_quad_.reset(new scm::gl::quad_geometry(
      context.render_device, math::vec2(-1.0f, -1.0f), math::vec2(1.0f, 1.0f)));

  ///////////////////////////////////////////////////////////////////////FBOs
  ///Create FBOs

  depth_pass_result_fbo_ = context.render_device->create_frame_buffer();
  accumulation_pass_result_fbo_ = context.render_device->create_frame_buffer();
  normalization_pass_result_fbo_ = context.render_device->create_frame_buffer();

  update_textures(context);

  ///////////////////////////////////////////////////////////////////////FBOs
  ///END
  //****
  ///////////////////////////////////////////////////////////////////////STATES
  ///BEGIN
  linear_sampler_state_ = context.render_device->create_sampler_state(
      scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);

  no_depth_test_depth_stencil_state_ = context.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_ALWAYS);

  depth_test_without_writing_depth_stencil_state_ = context.render_device
      ->create_depth_stencil_state(true, false, scm::gl::COMPARISON_LESS);

  color_accumulation_state_ =
      context.render_device->create_blend_state(true,
                                                scm::gl::FUNC_ONE,
                                                scm::gl::FUNC_ONE,
                                                scm::gl::FUNC_ONE,
                                                scm::gl::FUNC_ONE,
                                                scm::gl::EQ_FUNC_ADD,
                                                scm::gl::EQ_FUNC_ADD);

  change_point_size_in_shader_state_ = context.render_device
      ->create_rasterizer_state(scm::gl::FILL_SOLID,
                                scm::gl::CULL_NONE,
                                scm::gl::ORIENT_CCW,
                                false,
                                false,
                                0.0,
                                false,
                                false,
                                scm::gl::point_raster_state(true));
  ///////////////////////////////////////////////////////////////////////STATES
  ///END
  //****
  ///////////////////////////////////////////////////////////////////////MISC
  ///BEGIN
  material_id_ = 0;
  last_geometry_state_ = invalid_state;
  ///////////////////////////////////////////////////////////////////////MISC_END
  //****
  ///////////////////////////////////////////////////////////////////////PBR
  ///RELATED BEGIN
  pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

  size_t size_of_node_in_bytes =
      database->size_of_surfel() * database->surfels_per_node();

  size_t upload_budget_in_nodes =
      (policy->upload_budget_in_mb() * 1024u * 1024u) / size_of_node_in_bytes;

  unsigned long long sizeOfNode =
      database->surfels_per_node() * database->size_of_surfel();
  unsigned long long sizeOfTempBuffers = upload_budget_in_nodes * sizeOfNode;

  temp_buffer_A_ =
      context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_COPY,
                                           sizeOfTempBuffers,
                                           NULL);
  temp_buffer_B_ =
      context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_COPY,
                                           sizeOfTempBuffers,
                                           NULL);

  size_t render_budget_in_nodes =
      (policy->render_budget_in_mb() * 1024u * 1024u) / size_of_node_in_bytes;

  unsigned long long sizeOfRenderBuffer = render_budget_in_nodes * sizeOfNode;

  render_buffer_ =
      context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_COPY,
                                           sizeOfRenderBuffer);

  vertex_array_ = context.render_device->create_vertex_array(
      scm::gl::vertex_format(0,
                             0,
                             scm::gl::TYPE_VEC3F,
                             sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0,
          1,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          2,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          3,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          4,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          5,
          scm::gl::TYPE_FLOAT,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0,
          6,
          scm::gl::TYPE_VEC3F,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel)),
      boost::assign::list_of(render_buffer_));

  temp_buffer_A_is_mapped_ = false;
  temp_buffer_B_is_mapped_ = false;
  mapped_temp_buffer_A_ = nullptr;
  mapped_temp_buffer_B_ = nullptr;

  previous_framecount_ = -1;

  return upload_succeeded;
}

////////////////////////////////////////////////////////////////////////////////

GeometryUberShader::stage_mask PLODUberShader::get_stage_mask() const {
  return GeometryUberShader::PRE_FRAME_STAGE |
         GeometryUberShader::PRE_DRAW_STAGE |
         GeometryUberShader::POST_DRAW_STAGE |
         GeometryUberShader::POST_FRAME_STAGE;
}

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::preframe(RenderContext const& ctx) const {

  upload_to(ctx);

  if (previous_framecount_ != ctx.framecount) {
    previous_framecount_ = ctx.framecount;

    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
    pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();

    controller->ResetSystem();

    pbr::context_t context_id = controller->DeduceContextId(ctx.id);

    if (controller->IsSystemResetSignaled(context_id)) {
      reset(ctx);
    }

    if (!controller->TemporaryBuffersAvailable(context_id)) {
      controller->StoreTemporaryBuffers(
          context_id,
          get_mapped_temp_buffer_ptr(
              ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A),
          get_mapped_temp_buffer_ptr(
              ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B));
    }

    //swap cut database
    cuts->AcceptFront(context_id);

    //run cut update
    controller->DispatchCutUpdate(context_id);

    if (cuts->IsFrontModified(context_id)) {
      //upload data to gpu

      unmap_temp_buffer_ptr(ctx, cuts->GetBuffer(context_id));

      copy_to_main_memory(ctx, cuts->GetBuffer(context_id));

      cuts->SignalModificationComplete(context_id);

      controller->StoreTemporaryBuffers(
          context_id,
          get_mapped_temp_buffer_ptr(
              ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A),
          get_mapped_temp_buffer_ptr(
              ctx, pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B));
    }
  }

  last_geometry_state_ = pre_frame_state;

  {
    //clear depth FBOS depth attachments
    ctx.render_context->clear_depth_stencil_buffer(depth_pass_result_fbo_);
    ctx.render_context->clear_color_buffer(
        depth_pass_result_fbo_, 0, scm::math::vec4f(1.0f, 1.0f, 1.0f, 1.0f));

    //clear_accumulation FBOS color attachment
    ctx.render_context
        ->clear_color_buffer(accumulation_pass_result_fbo_,
                             0,
                             scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

    ctx.render_context
        ->clear_color_buffer(accumulation_pass_result_fbo_,
                             1,
                             scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));

    //clear normalization FBOs color attachment
    ctx.render_context
        ->clear_color_buffer(normalization_pass_result_fbo_,
                             0,
                             scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
    ctx.render_context
        ->clear_color_buffer(normalization_pass_result_fbo_,
                             1,
                             scm::math::vec3f(0.0f, 0.0f, 0.0f));
  }

  //clear cached frustum culling results for all models
  model_frustum_culling_results_.clear();
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void PLODUberShader::predraw(RenderContext const& ctx,
                                         std::string const& file_name,
                                         std::string const& material_name,
                                         scm::math::mat4 const& model_matrix,
                                         scm::math::mat4 const& normal_matrix,
                                         Frustum const& frustum,
                                         View const& view) const {

  //determine frustum parameters
  std::vector<math::vec3> frustum_corner_values = frustum.get_corners();
  float top_minus_bottom = scm::math::length((frustum_corner_values[2]) -
                                             (frustum_corner_values[0]));
  float height_divided_by_top_minus_bottom =
      (render_window_dims_)[1] / top_minus_bottom;
  float near_plane_value = frustum.get_clip_near();
  float far_plane_value = frustum.get_clip_far();

  if (last_geometry_state_ != pre_draw_state) {
    context_guard_ =
        std::make_shared<scm::gl::context_all_guard>(ctx.render_context);

    //enable dynamic point size in shaders
    ctx.render_context
        ->set_rasterizer_state(change_point_size_in_shader_state_);

    //bind fbo
    ctx.render_context->set_frame_buffer(depth_pass_result_fbo_);

    get_program(depth_pass)->set_uniform(ctx,
                                         height_divided_by_top_minus_bottom,
                                         "height_divided_by_top_minus_bottom");
    get_program(depth_pass)->set_uniform(ctx, near_plane_value, "near_plane");
    get_program(depth_pass)->set_uniform(
        ctx, (far_plane_value - near_plane_value), "far_minus_near_plane");
    last_geometry_state_ = pre_draw_state;
  }

  pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
  pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();

  pbr::context_t context_id = controller->DeduceContextId(ctx.id);
  pbr::view_t view_id = controller->DeduceViewId(context_id, view.id);
  pbr::model_t model_id = controller->DeduceModelId(file_name);

  //send camera and model_matrix to cut update
  pbr::ren::Camera cut_update_cam(
      view_id, near_plane_value, frustum.get_view(), frustum.get_projection());

  cuts->SendCamera(context_id, view_id, cut_update_cam);
  cuts->SendHeightDividedByTopMinusBottom(
      context_id, view_id, height_divided_by_top_minus_bottom);
  cuts->SendTransform(context_id, model_id, model_matrix);

  pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
  std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();

  //calculate frustum culling results
  pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

  pbr::ren::KdnTree const* kdn_tree = database->GetModel(model_id)->kdn_tree();

  scm::gl::frustum culling_frustum =
      cut_update_cam.GetFrustumByModel(model_matrix);

  std::vector<scm::gl::boxf> const& model_bounding_boxes =
      kdn_tree->bounding_boxes();

  model_frustum_culling_results_[model_id].clear();
  model_frustum_culling_results_[model_id].resize(node_list.size());

  unsigned int node_counter = 0;
  for (const auto& n : node_list) {
    ++node_counter;

    //0 = completely in frustum, 1 = completely outside frustum, 2 = intersecting frustum
    bool is_in_frustum = culling_frustum.classify(model_bounding_boxes[n.node_id_]) != 1;
    model_frustum_culling_results_[model_id][node_counter] = is_in_frustum;
  }

  auto plod_ressource = std::static_pointer_cast<PLODRessource>(
      GeometryDatabase::instance()->lookup(file_name));
  auto material = MaterialDatabase::instance()->lookup(material_name);

  //////////////////////////////
  //DEPTH PASS (first) BEGIN
  /////////////////////////////
  scm::math::vec4f x_unit_vec(1.0, 0.0, 0.0, 0.0);
  float radius_model_scaling = scm::math::length(model_matrix * x_unit_vec);

  get_program(depth_pass)
      ->set_uniform(ctx, radius_model_scaling, "radius_model_scaling");

  get_program(depth_pass)
      ->set_uniform(ctx,
                    transpose(inverse(frustum.get_view() * model_matrix)),
                    "gua_normal_matrix");
  get_program(depth_pass)->set_uniform(ctx, model_matrix, "gua_model_matrix");

  if (material && plod_ressource) {
    get_program(depth_pass)->use(ctx);
    plod_ressource->draw(ctx,
                         context_id,
                         view_id,
                         model_id,
                         vertex_array_,
                         model_frustum_culling_results_[model_id]);
    get_program(depth_pass)->unuse(ctx);
  }

}

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::draw(RenderContext const& ctx,
                          std::string const& file_name,
                          std::string const& material_name,
                          scm::math::mat4 const& model_matrix,
                          scm::math::mat4 const& normal_matrix,
                          Frustum const& frustum,
                          View const& view) const {
  throw std::runtime_error("PLODUberShader::draw(): not implemented");
}

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::postdraw(RenderContext const& ctx,
                              std::string const& file_name,
                              std::string const& material_name,
                              scm::math::mat4 const& model_matrix,
                              scm::math::mat4 const& normal_matrix,
                              Frustum const& frustum,
                              View const& view) const {

  auto plod_ressource = std::static_pointer_cast<PLODRessource>(
      GeometryDatabase::instance()->lookup(file_name));
  auto material = MaterialDatabase::instance()->lookup(material_name);

  float near_plane_value = frustum.get_clip_near();
  float far_plane_value = frustum.get_clip_far();

  if (last_geometry_state_ != post_draw_state) {
    (context_guard_).reset();
    context_guard_ =
        std::make_shared<scm::gl::context_all_guard>(ctx.render_context);
    //destroy old context guard and create new one

    //enable dynamic point size in shaders
    ctx.render_context
        ->set_rasterizer_state(change_point_size_in_shader_state_);

    //disable depth test
    //ctx.render_context
    //    ->set_depth_stencil_state(no_depth_test_depth_stencil_state_);

    ctx.render_context
        ->set_depth_stencil_state(depth_test_without_writing_depth_stencil_state_);

    //set blend state to accumulate
    ctx.render_context->set_blend_state(color_accumulation_state_);

    // bind accumulation FBO
    ctx.render_context->set_frame_buffer(accumulation_pass_result_fbo_);

    //attach linear depth buffer of depth past for early-z rejection
    accumulation_pass_result_fbo_->attach_depth_stencil_buffer(depth_pass_linear_depth_result_);

    std::vector<math::vec3> corner_values = frustum.get_corners();

    float top_minus_bottom =
        scm::math::length((corner_values[2]) - (corner_values[0]));

    float height_divided_by_top_minus_bottom =
        (render_window_dims_)[1] / top_minus_bottom;

    get_program(accumulation_pass)
        ->set_uniform(ctx,
                      height_divided_by_top_minus_bottom,
                      "height_divided_by_top_minus_bottom");
    get_program(accumulation_pass)
        ->set_uniform(ctx, near_plane_value, "near_plane");
    get_program(accumulation_pass)->set_uniform(
        ctx, (far_plane_value - near_plane_value), "far_minus_near_plane");

    get_program(accumulation_pass)
        ->set_uniform(ctx, math::vec2(render_window_dims_), "win_dims");

    last_geometry_state_ = post_draw_state;

  }

  scm::math::vec4f x_unit_vec(1.0f, 0.f, 0.f, 0.f);

  float radius_model_scaling = scm::math::length(model_matrix * x_unit_vec);

  get_program(accumulation_pass)
      ->set_uniform(ctx, radius_model_scaling, "radius_model_scaling");

  ctx.render_context
      ->bind_texture(depth_pass_linear_depth_result_, linear_sampler_state_, 0);
  get_program(accumulation_pass)->get_program(ctx)
      ->uniform_sampler("p01_depth_texture", 0);

  get_program(accumulation_pass)
      ->set_uniform(ctx,
                    transpose(inverse(frustum.get_view() * model_matrix)),
                    "gua_normal_matrix");
  get_program(accumulation_pass)
      ->set_uniform(ctx, model_matrix, "gua_model_matrix");
 
  get_program(accumulation_pass)
      ->set_uniform(ctx,
                    transpose(inverse(model_matrix)),
                    "transposed_inverse_model_matrix");

  if (material && plod_ressource) {
    pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::context_t context_id = controller->DeduceContextId(ctx.id);
    pbr::model_t model_id = controller->DeduceModelId(file_name);
    pbr::view_t view_id = controller->DeduceViewId(context_id, view.id);

    material_id_ = material->get_id();

    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();
    pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
    std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list =
        cut.complete_set();

    //calculate frustum culling results
    pbr::ren::Camera cut_update_cam(view_id,
                                    near_plane_value,
                                    frustum.get_view(),
                                    frustum.get_projection());

    pbr::ren::KdnTree const* kdn_tree =
        database->GetModel(model_id)->kdn_tree();

    scm::gl::frustum culling_frustum =
        cut_update_cam.GetFrustumByModel(model_matrix);

    std::vector<scm::gl::boxf> const& model_bounding_boxes =
        kdn_tree->bounding_boxes();

    get_program(accumulation_pass)->use(ctx);
    plod_ressource->draw(ctx,
                         context_id,
                         view_id,
                         model_id,
                         vertex_array_,
                         model_frustum_culling_results_[model_id]);
    get_program(accumulation_pass)->unuse(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void PLODUberShader::postframe(RenderContext const& ctx) const {
  (context_guard_).reset();

  bool using_default_pbr_material = true;
  {

    {
      scm::gl::context_all_guard guard(ctx.render_context);
      // bind normalization FBO
      ctx.render_context->set_frame_buffer(normalization_pass_result_fbo_);

      get_program(normalization_pass)->use(ctx);

      get_program(normalization_pass)->set_uniform(
          ctx, using_default_pbr_material, "using_default_pbr_material");

      //bind color outputs to be normalized for third pass
      ctx.render_context->bind_texture(
          accumulation_pass_color_result_, linear_sampler_state_, 0);
      get_program(normalization_pass)->get_program(ctx)
          ->uniform_sampler("p02_color_texture", 0);

      ctx.render_context->bind_texture(
          accumulation_pass_normal_result_, linear_sampler_state_, 1);
      get_program(normalization_pass)->get_program(ctx)
          ->uniform_sampler("p02_normal_texture", 1);

      fullscreen_quad_->draw(ctx.render_context);
      get_program(normalization_pass)->unuse(ctx);

    }

    {
      scm::gl::context_all_guard guard(ctx.render_context);

      get_program(reconstruction_pass)->use(ctx);
      get_program(reconstruction_pass)
          ->set_uniform(ctx, material_id_, "gua_material_id");
      get_program(reconstruction_pass)->set_uniform(
          ctx, using_default_pbr_material, "using_default_pbr_material");
      get_program(reconstruction_pass)
          ->set_uniform(ctx, math::vec2(render_window_dims_), "win_dims");

      //bind logarithmic depth texture for gbuffer
      ctx.render_context->bind_texture(
          depth_pass_log_depth_result_, linear_sampler_state_, 0);
      get_program(reconstruction_pass)->get_program(ctx)
          ->uniform_sampler("p01_depth_texture", 0);

      //bind color output for gbuffer
      ctx.render_context->bind_texture(
          normalization_pass_color_result_, linear_sampler_state_, 1);
      get_program(reconstruction_pass)->get_program(ctx)
          ->uniform_sampler("p02_color_texture", 1);

      //bind normal output for gbuffer
      ctx.render_context->bind_texture(
          normalization_pass_normal_result_, linear_sampler_state_, 2);
      get_program(reconstruction_pass)->get_program(ctx)
          ->uniform_sampler("p02_normal_texture", 2);

      fullscreen_quad_->draw(ctx.render_context);
      get_program(reconstruction_pass)->unuse(ctx);

    }
  }
}

////////////////////////////////////////////////////////////////////////////////

char* PLODUberShader::get_mapped_temp_buffer_ptr(
    RenderContext const& ctx,
    pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const {
  switch (buffer) {
    case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A:
      if (!temp_buffer_A_is_mapped_) {
        mapped_temp_buffer_A_ = (char*)ctx.render_context
            ->map_buffer(temp_buffer_A_, scm::gl::ACCESS_READ_WRITE);
        temp_buffer_A_is_mapped_ = true;
      }
      return mapped_temp_buffer_A_;
      break;

    case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B:
      if (!temp_buffer_B_is_mapped_) {
        mapped_temp_buffer_B_ = (char*)ctx.render_context
            ->map_buffer(temp_buffer_B_, scm::gl::ACCESS_READ_WRITE);
        temp_buffer_B_is_mapped_ = true;
      }
      return mapped_temp_buffer_B_;
      break;

    default:
      break;
  }

  Logger::LOG_ERROR << "Failed to map temporary buffer." << std::endl;

  return nullptr;
}

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::unmap_temp_buffer_ptr(
    RenderContext const& ctx,
    pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const {
  switch (buffer) {
    case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A:
      if (temp_buffer_A_is_mapped_) {
        ctx.render_context->unmap_buffer(temp_buffer_A_);
        temp_buffer_A_is_mapped_ = false;
      }
      break;

    case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B:
      if (temp_buffer_B_is_mapped_) {
        ctx.render_context->unmap_buffer(temp_buffer_B_);
        temp_buffer_B_is_mapped_ = false;
      }
      break;

    default:
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////

void PLODUberShader::copy_to_main_memory(
    RenderContext const& ctx,
    pbr::ren::CutDatabaseRecord::TemporaryBuffer const& buffer) const {
  pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();

  size_t size_of_node_in_bytes =
      database->surfels_per_node() * database->size_of_surfel();

  pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();

  pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();

  pbr::context_t context_id = controller->DeduceContextId(ctx.id);

  switch (buffer) {
    case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A: {
      if (temp_buffer_A_is_mapped_) {
        Logger::LOG_ERROR << "Failed to transfer nodes into main memory. "
                          << "Temp Storage A was still mapped." << std::endl;
        assert(false);
      }
      std::vector<pbr::ren::CutDatabaseRecord::SlotUpdateDescr>&
          transfer_descr_list = cuts->GetUpdatedSet(context_id);

      if (!transfer_descr_list.empty()) {
        for (const auto& transfer_desc : transfer_descr_list) {
          size_t offset_in_temp_VBO =
              transfer_desc.src_ * size_of_node_in_bytes;
          size_t offset_in_render_VBO =
              transfer_desc.dst_ * size_of_node_in_bytes;
          ctx.render_context->copy_buffer_data(render_buffer_,
                                               temp_buffer_A_,
                                               offset_in_render_VBO,
                                               offset_in_temp_VBO,
                                               size_of_node_in_bytes);
        }
      }
      break;
    }

    case pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B: {
      if (temp_buffer_B_is_mapped_) {
        Logger::LOG_ERROR << "Failed to transfer nodes into main memory. "
                          << "Temp Storage B was still mapped." << std::endl;
        assert(false);
      }
      std::vector<pbr::ren::CutDatabaseRecord::SlotUpdateDescr>&
          transfer_descr_list = cuts->GetUpdatedSet(context_id);
      if (!transfer_descr_list.empty()) {
        for (const auto& transfer_desc : transfer_descr_list) {
          size_t offset_in_temp_VBO =
              transfer_desc.src_ * size_of_node_in_bytes;
          size_t offset_in_render_VBO =
              transfer_desc.dst_ * size_of_node_in_bytes;
          ctx.render_context->copy_buffer_data(render_buffer_,
                                               temp_buffer_B_,
                                               offset_in_render_VBO,
                                               offset_in_temp_VBO,
                                               size_of_node_in_bytes);
        }
      }
      break;
    }

    default:
      break;
  }

}

////////////////////////////////////////////////////////////////////////////////
void PLODUberShader::reset(RenderContext const& context) const {
 /* unmap_temp_buffer_ptr(context,
                        pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_A);
  unmap_temp_buffer_ptr(context,
                        pbr::ren::CutDatabaseRecord::TemporaryBuffer::BUFFER_B);

  temp_buffer_A_.reset();
  temp_buffer_B_.reset();
  render_buffer_.reset();
  vertex_array_.reset();

  pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
  pbr::ren::Policy* policy = pbr::ren::Policy::GetInstance();

  size_t size_of_node_in_bytes =
      database->size_of_surfel() * database->surfels_per_node();

  size_t upload_budget_in_nodes =
      (policy->upload_budget_in_mb() * 1024u * 1024u) / size_of_node_in_bytes;

  size_t render_budget_in_nodes =
      (policy->render_budget_in_mb() * 1024u * 1024u) / size_of_node_in_bytes;

  unsigned long long sizeOfTempBuffers =
      upload_budget_in_nodes * size_of_node_in_bytes;
  temp_buffer_A_ =
      context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_COPY,
                                           sizeOfTempBuffers,
                                           NULL);
  temp_buffer_B_ =
      context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_COPY,
                                           sizeOfTempBuffers,
                                           NULL);

  size_t stride_size = sizeof(unsigned int) + sizeof(float) + 6 * sizeof(float);

  unsigned long long sizeOfRenderBuffer =
      render_budget_in_nodes * size_of_node_in_bytes;
  render_buffer_ =
      context.render_device->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                                           scm::gl::USAGE_DYNAMIC_COPY,
                                           sizeOfRenderBuffer);

  vertex_array_ = context.render_device->create_vertex_array(
      scm::gl::vertex_format(0,
                             0,
                             scm::gl::TYPE_VEC3F,
                             sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0,
          1,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          2,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          3,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          4,
          scm::gl::TYPE_UBYTE,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel),
          scm::gl::INT_FLOAT_NORMALIZE)(
          0,
          5,
          scm::gl::TYPE_FLOAT,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel))(
          0,
          6,
          scm::gl::TYPE_VEC3F,
          sizeof(pbr::ren::RawPointCloud::SerializedSurfel)),
      boost::assign::list_of(render_buffer_));

  temp_buffer_A_is_mapped_ = false;
  temp_buffer_B_is_mapped_ = false;
  mapped_temp_buffer_A_ = nullptr;
  mapped_temp_buffer_B_ = nullptr;
*/
}

////////////////////////////////////////////////////////////////////////////////

std::string PLODUberShader::default_plod_material_name() const {
  return "gua_pbr";
}

}
#endif
