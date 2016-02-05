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
#include <gua/OculusWindow.hpp>

// gua headers
#include <gua/utils/Logger.hpp>
#include <gua/databases/Resources.hpp>
#include <scm/gl_core/render_device/opengl/gl_core.h>

// external headers
#include <iostream>

namespace gua {

// ---------------------------------------------------------------
//'static member
// ---------------------------------------------------------------
bool OculusWindow::oculus_environment_initialized_ = false;
unsigned OculusWindow::registered_oculus_device_count_ = 0;

// ---------------------------------------------------------------
// static functions
// ---------------------------------------------------------------
// calling this once in the application before any window creation
// will set up the oculus environment
void OculusWindow::initialize_oculus_environment() {
  if( !oculus_environment_initialized_ ) {
      ovrResult result = ovr_Initialize(nullptr);
      oculus_environment_initialized_ = true;

      if (!OVR_SUCCESS(result)) {
        Logger::LOG_WARNING << "Failed to initialize oculus environment!" << "Errorcode:" << (int)result  << std::endl;
      }
  } else {
      Logger::LOG_WARNING << "Attempt to initialize oculus environment multiple times!" << std::endl;
  }
}

// calling this once in the application after the windows are destroyed will shutdown
// the oculus environment
void OculusWindow::shutdown_oculus_environment() {
  if( oculus_environment_initialized_ ) {
      ovr_Shutdown();
      oculus_environment_initialized_ = false;
  } else {
      Logger::LOG_WARNING << "Attempt to shutdown uninitialized oculus environment!"
                          << std::endl;
  }
}


// ---------------------------------------------------------------
// helper functions
// ---------------------------------------------------------------

#ifndef _WIN32
// gets the distortion meshes of the oculus API and initializes a parsing process to
// gua buffers
void OculusWindow::initialize_distortion_meshes(ovrHmd const& hmd, RenderContext const& ctx) {
  ovrEyeRenderDesc eyeRenderDesc[2];

  eyeRenderDesc[0] = ovrHmd_GetRenderDesc( hmd, ovrEye_Left, hmd->DefaultEyeFov[0] );
  eyeRenderDesc[1] = ovrHmd_GetRenderDesc( hmd, ovrEye_Right, hmd->DefaultEyeFov[1] );

  // Create the Distortion Meshes (one for each eye):
  OculusDistortionMesh distortion_mesh_cpu_buffer[2];


  for ( int eye_num = 0; eye_num < 2; eye_num ++ ) {
  ovrDistortionMesh meshData;
  ovrHmd_CreateDistortionMesh( hmd,
    eyeRenderDesc[eye_num].Eye,
    eyeRenderDesc[eye_num].Fov,
    0,
    &meshData );

  OVR::Sizei eyeTextureSize = OVR::Sizei(hmd->Resolution.w/2, hmd->Resolution.h);

  // retrieve parameters for the correct scaling and offset of the distortion mesh
  ovrRecti viewports[2];
  viewports[0].Pos.x = 0;
  viewports[0].Pos.y = 0;
  viewports[0].Size.w = eyeTextureSize.w;
  viewports[0].Size.h = eyeTextureSize.h;
  viewports[1].Pos.x = eyeTextureSize.w;
  viewports[1].Pos.y = 0;
  viewports[1].Size.w = eyeTextureSize.w;
  viewports[1].Size.h = eyeTextureSize.h;

  // transformation parameters for the distortion mesh
  ovrVector2f uv_scale_offset_[2];


  ovrHmd_GetRenderScaleAndOffset( eyeRenderDesc[eye_num].Fov,
    eyeTextureSize, viewports[eye_num],
    uv_scale_offset_);


  bool is_left_eye = 0 == eye_num;

  // actual creation of guacamole buffers in this helper object
  distortion_mesh_cpu_buffer[eye_num].initialize_distortion_mesh(meshData, uv_scale_offset_, is_left_eye);
  //  the distortion meshes are no lonber needed at this point
  ovrHmd_DestroyDistortionMesh( &meshData );

  // create the distortion mesh VBO
  distortion_mesh_vertices_[eye_num] =
    ctx.render_device
    ->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                    scm::gl::USAGE_STATIC_DRAW,
                    distortion_mesh_cpu_buffer[eye_num].num_vertices
                      * sizeof(OculusDistortionMesh::DistortionVertex),
                    0);

  // map it to cpu
  OculusDistortionMesh::DistortionVertex*
  data(static_cast<OculusDistortionMesh::DistortionVertex*>
    (ctx.render_context->map_buffer(distortion_mesh_vertices_[eye_num],
                      scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  // fill it with the pre-loaded cpu data
  distortion_mesh_cpu_buffer[eye_num].copy_to_buffer(data);

  // unmap it
  ctx.render_context->unmap_buffer(distortion_mesh_vertices_[eye_num]);

  // create the distortion mesh IBO
  distortion_mesh_indices_[eye_num] =
    ctx.render_device
      ->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                      scm::gl::USAGE_STATIC_DRAW,
                      distortion_mesh_cpu_buffer[eye_num].num_indices
                        * sizeof(unsigned),
                      &(distortion_mesh_cpu_buffer[eye_num].indices[0]) );


  distortion_mesh_vertex_array_[eye_num] =
    ctx.render_device->create_vertex_array(
                       distortion_mesh_cpu_buffer[eye_num].get_vertex_format(),
                       {distortion_mesh_vertices_[eye_num]});

  ctx.render_context->apply_vertex_input();

  // store the number of indices for rendering the distortion mesh
  num_distortion_mesh_indices_[eye_num] = distortion_mesh_cpu_buffer[eye_num].num_indices;
  }
}
#endif


OculusWindow::OculusWindow(std::string const& display):
  GlfwWindow() {

  initialize_oculus_environment();

#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6

  // automatically register the HMDs in order
#ifdef _WIN32
    ovrHmd_Create(registered_oculus_device_count_++, &registered_HMD_);
    // register all three sensors for sensor fusion
    bool tracking_configured = ovrHmd_ConfigureTracking(registered_HMD_, ovrTrackingCap_Orientation
        | ovrTrackingCap_MagYawCorrection
        | ovrTrackingCap_Position, 0) == ovrSuccess;

    ovrHmd_SetEnabledCaps(registered_HMD_, ovrHmdCap_LowPersistence | ovrHmdCap_DynamicPrediction);
#else
    registered_HMD_ = ovrHmd_Create(registered_oculus_device_count_++);
    // register all three sensors for sensor fusion
    bool tracking_configured = ovrHmd_ConfigureTracking(registered_HMD_, ovrTrackingCap_Orientation
        | ovrTrackingCap_MagYawCorrection
        | ovrTrackingCap_Position, 0);
    
    std::string oculus_product_id (registered_HMD_->ProductName);
    has_vertical_display_ = oculus_product_id.find( "DK1" ) == std::string::npos;
#endif

  if (!tracking_configured) {
      ovrHmd_Destroy(registered_HMD_);
      registered_HMD_ = NULL;
      gua::Logger::LOG_WARNING << "The oculus device does not support demanded tracking feature.\n";

      exit(-1);
  }

  config.set_size(math::vec2ui(registered_HMD_->Resolution.w, registered_HMD_->Resolution.h));
  config.set_left_resolution(math::vec2ui(registered_HMD_->Resolution.w/2, registered_HMD_->Resolution.h));
  config.set_left_position(math::vec2ui(0, 0));
  config.set_right_resolution(math::vec2ui(registered_HMD_->Resolution.w/2, registered_HMD_->Resolution.h));
  config.set_right_position(math::vec2ui(registered_HMD_->Resolution.w/2, 0));

#ifdef _WIN32
  // Initialize VR structures, filling out description.
  ovrEyeRenderDesc eyeRenderDesc[2];
  ovrVector3f      hmdToEyeViewOffset[2];
  eyeRenderDesc[0] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_->DefaultEyeFov[0]);
  eyeRenderDesc[1] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_->DefaultEyeFov[1]);
  hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeViewOffset;
  hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeViewOffset;

  // Initialize our single full screen Fov layer.
  color_layer_.Header.Type = ovrLayerType_EyeFov;
  color_layer_.Header.Flags = 0;
  color_layer_.Fov[0] = eyeRenderDesc[0].Fov;
  color_layer_.Fov[1] = eyeRenderDesc[1].Fov;

  ovrRecti left_viewport;
  left_viewport.Size = { config.left_resolution().x, config.left_resolution().y };
  left_viewport.Pos = { config.left_position().x, config.left_position().y };

  ovrRecti right_viewport;
  right_viewport.Size = { config.right_resolution().x, config.right_resolution().y };
  right_viewport.Pos = { config.right_position().x, config.right_position().y };

  color_layer_.Viewport[0] = left_viewport;
  color_layer_.Viewport[1] = right_viewport;

#endif

#else

  try {
    ovrGraphicsLuid luid;
    ovrResult result = ovr_Create(&registered_HMD_, &luid);

    if (!OVR_SUCCESS(result)) {
      throw std::runtime_error("Unable to create HMD.");
    }

    registered_HMD_desc_ = ovr_GetHmdDesc(registered_HMD_);
    
    result = ovr_ConfigureTracking(registered_HMD_, ovrTrackingCap_Orientation
      | ovrTrackingCap_MagYawCorrection
      | ovrTrackingCap_Position, 0) == ovrSuccess;

    if (!OVR_SUCCESS(result)) {
      ovr_Destroy(registered_HMD_);
      registered_HMD_ = 0;
      Logger::LOG_WARNING << "Unable to initialize HMD tracking. Errorcode:" << (int)result << std::endl;
      throw std::runtime_error("Unable to initialize HMD tracking.");
    }

    // get optimal texture size for rendering
    ovrSizei ideal_texture_size_left  = ovr_GetFovTextureSize(registered_HMD_, ovrEyeType(0), registered_HMD_desc_.DefaultEyeFov[0], 1);
    ovrSizei ideal_texture_size_right = ovr_GetFovTextureSize(registered_HMD_, ovrEyeType(1), registered_HMD_desc_.DefaultEyeFov[1], 1);

    math::vec2ui window_size(ideal_texture_size_left.w + ideal_texture_size_right.w, std::max(ideal_texture_size_left.h, ideal_texture_size_right.h));

    // initialize window => resolution is independent of rendering resolution!
    config.set_size(window_size);

    config.set_left_resolution(math::vec2ui(ideal_texture_size_left.w, ideal_texture_size_left.h));
    config.set_left_position(math::vec2ui(0, 0));
    config.set_right_resolution(math::vec2ui(ideal_texture_size_right.w, ideal_texture_size_right.h));
    config.set_right_position(math::vec2ui(ideal_texture_size_left.w, 0));

    // Initialize VR structures, filling out description.
    ovrEyeRenderDesc eyeRenderDesc[2];
    ovrVector3f      hmdToEyeViewOffset[2];

    eyeRenderDesc[0] = ovr_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_desc_.DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovr_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_desc_.DefaultEyeFov[1]);

    hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeViewOffset;
    hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeViewOffset;

    // Initialize our single full screen Fov layer.
    color_layer_.Header.Type = ovrLayerType_EyeFov;
    color_layer_.Header.Flags = 0;
    color_layer_.Fov[0] = eyeRenderDesc[0].Fov;
    color_layer_.Fov[1] = eyeRenderDesc[1].Fov;

    ovrRecti left_viewport;
    left_viewport.Size = { config.left_resolution().x, config.left_resolution().y };
    left_viewport.Pos = { config.left_position().x, config.left_position().y };

    ovrRecti right_viewport;
    right_viewport.Size = { config.right_resolution().x, config.right_resolution().y };
    right_viewport.Pos = { config.right_position().x, config.right_position().y };

    color_layer_.Viewport[0] = left_viewport;
    color_layer_.Viewport[1] = right_viewport;

  }
  catch (std::exception& e) {
    gua::Logger::LOG_WARNING << "Failed to initialize oculus rift.\n" << e.what() << std::endl;
  }
#endif
  config.set_title("guacamole");
  config.set_display_name(display);
  config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);

  this->calculate_viewing_setup();

}

////////////////////////////////////////////////////////////////////////////////

OculusWindow::~OculusWindow() {
  // cleanup the oculus device
#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
  ovrHmd_Destroy(registered_HMD_);
#else
  ovr_Destroy(registered_HMD_);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::init_context() {

#ifdef _WIN32
  WindowBase::init_context();

  auto const& glapi = ctx_.render_context->opengl_api();

  glapi.glGenFramebuffers(1, &blit_fbo_read_);
  glapi.glGenFramebuffers(1, &blit_fbo_write_);

#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
  if (ovrHmd_CreateSwapTextureSetGL(registered_HMD_, scm::gl::FORMAT_SRGBA_8,
      config.left_resolution().x + config.right_resolution().x,
      std::max(config.left_resolution().y, config.right_resolution().y), &swap_texures_) != ovrSuccess) {
#else
  ovrResult result = ovr_CreateSwapTextureSetGL(registered_HMD_, scm::gl::FORMAT_SRGBA_8,
                                          config.left_resolution().x + config.right_resolution().x,
                                          std::max(config.left_resolution().y, config.right_resolution().y),
                                          &swap_texures_);
  if (!OVR_SUCCESS(result)) {
#endif
      gua::Logger::LOG_WARNING << "Failed to create swap textures for oculus rift.\n";
  }

  color_layer_.ColorTexture[0] = swap_texures_;
  color_layer_.ColorTexture[1] = swap_texures_;

#else

    ctx_.render_device  = scm::gl::render_device_ptr(new scm::gl::render_device());
    ctx_.render_context = ctx_.render_device->main_context();

    {
        std::lock_guard<std::mutex> lock(last_context_id_mutex_);
        ctx_.id = last_context_id_++;
    }

    ctx_.render_window = this;

    if (config.get_debug()) {
        ctx_.render_context->register_debug_callback(boost::make_shared<DebugOutput>());
    }

  fullscreen_shader_.create_from_sources(R"(
      #version 420
      #extension GL_NV_bindless_texture : require

      layout(location=0) in vec2 in_position;
      layout(location=1) in vec2 in_tex_coords_r;
      layout(location=2) in vec2 in_tex_coords_g;
      layout(location=3) in vec2 in_tex_coords_b;
      layout(location=4) in float in_vignette_fac;

      out vec2 tex_coord_r;
      out vec2 tex_coord_g;
      out vec2 tex_coord_b;
      out float vig_factor;

      void main() {
          tex_coord_r = in_tex_coords_r;
          tex_coord_g = in_tex_coords_g;
          tex_coord_b = in_tex_coords_b;
          vig_factor = in_vignette_fac;
          gl_Position = vec4(in_position, 0.0, 1.0);
      }
    )", R"(
      #version 420
      #extension GL_NV_bindless_texture : require
      #extension GL_NV_gpu_shader5      : enable

      in vec2 tex_coord_r;
      in vec2 tex_coord_g;
      in vec2 tex_coord_b;
      in float vig_factor;

      uniform uvec2 sampler;

      layout (location = 0) out vec3 out_color;

      sampler2D get_tex(uvec2 handle) {
          return sampler2D(uint64_t(handle.x) + uint64_t(handle.y) * 4294967295);
      }

      vec3 get_color() {
          float red_component   = texture2D( get_tex(sampler), tex_coord_r).r;
          float green_component = texture2D( get_tex(sampler), tex_coord_g).g;
          float blue_component  = texture2D( get_tex(sampler), tex_coord_b).b;

          return vec3(red_component, green_component, blue_component);
      }

      void main() {
          out_color = vig_factor * get_color();
      }
    )");

  initialize_distortion_meshes(registered_HMD_, *get_context());

  depth_stencil_state_ = ctx_.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

  blend_state_ = ctx_.render_device->create_blend_state(true,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE);

  //disable backface culling for the distortion meshes
  no_backface_culling_state_ = ctx_.render_device->create_rasterizer_state(scm::gl::FILL_SOLID,
                                                                           scm::gl::CULL_NONE,
                                                                           scm::gl::ORIENT_CCW,
                                                                           true);
#endif
}

////////////////////////////////////////////////////////////////////////////////

void OculusWindow::display(std::shared_ptr<Texture> const& texture, bool is_left) {

#ifdef _WIN32

    auto const& glapi = ctx_.render_context->opengl_api();

    // setup draw buffer
    ovrGLTexture* tex = (ovrGLTexture*)&swap_texures_->Textures[swap_texures_->CurrentIndex];
    glapi.glBindTexture(GL_TEXTURE_2D, tex->OGL.TexId);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_fbo_write_);
    glapi.glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex->OGL.TexId, 0);
    //glapi.glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

    GLenum status = glapi.glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
      gua::Logger::LOG_WARNING << "Incomplete.\n";
    }

    // setup read buffer
    glapi.glBindTexture(GL_TEXTURE_2D, texture->get_buffer(ctx_)->object_id());
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_fbo_read_);
    glapi.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, texture->get_buffer(ctx_)->object_id(), 0);
    //glapi.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);

    status = glapi.glCheckFramebufferStatus(GL_READ_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
      gua::Logger::LOG_WARNING << "Incomplete.\n";
    }

    if (is_left) {
      glapi.glBlitFramebuffer(0, texture->height(), texture->width(), 0,
        config.left_position().x, config.left_position().y,
        config.left_resolution().x, config.left_resolution().y,
        GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }
    else {
      glapi.glBlitFramebuffer(0, texture->height(), texture->width(), 0,
        config.right_position().x, config.right_position().y,
        config.right_position().x + config.right_resolution().x, config.right_resolution().y,
        GL_COLOR_BUFFER_BIT, GL_NEAREST);
    }

    glapi.glBindTexture(GL_TEXTURE_2D, 0);
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0);

    WindowBase::display(texture, is_left);

#else

  unsigned current_eye_num = is_left ? 0 : 1;

  // use the distortion mesh shader as "fullscreen" shader
  fullscreen_shader_.use(*get_context());
  // upload the usual quad texture to be mapped on the distortion mesh
  fullscreen_shader_.set_uniform(*get_context(), texture->get_handle(ctx_), "sampler");

  get_context()->render_context->set_depth_stencil_state(depth_stencil_state_);
  get_context()->render_context->set_rasterizer_state(no_backface_culling_state_);

  // bind vertex array for the currently rendered eye
  get_context()->render_context->bind_vertex_array(distortion_mesh_vertex_array_[current_eye_num]);
  get_context()->render_context->bind_index_buffer(distortion_mesh_indices_[current_eye_num],
    scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  get_context()->render_context->apply();

  // actually draw the distortion mesh
  get_context()->render_context->draw_elements(num_distortion_mesh_indices_[current_eye_num]);

  get_context()->render_context->reset_state_objects();

  fullscreen_shader_.unuse(*get_context());

  ovrHmd_EndFrameTiming(registered_HMD_);

#endif
}

math::vec2 const OculusWindow::get_left_screen_size() const {
  return screen_size_[0];
}

math::vec2 const OculusWindow::get_right_screen_size() const {
  return screen_size_[1];
}

math::vec3 const OculusWindow::get_left_screen_translation() const {
  return screen_translation_[0];
}

math::vec3 const OculusWindow::get_right_screen_translation() const {
  return screen_translation_[1];
}

float const OculusWindow::get_IPD() const {
  // get the interpupillary distance defined by the oculus rift config tool.
  // note: the oculus runtime below 0.6 has a bug that causes the IPD not to change.
  //        therefore one is bound to use the returned 6.4 cm

  ovrEyeRenderDesc eyeRenderDesc[2];
  
#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
  eyeRenderDesc[0] 
    = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_->DefaultEyeFov[0]);
  eyeRenderDesc[1] 
    = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_->DefaultEyeFov[1]);
#else
  eyeRenderDesc[0] = ovr_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_desc_.DefaultEyeFov[0]);
  eyeRenderDesc[1] = ovr_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_desc_.DefaultEyeFov[1]);
#endif

  ovrVector3f hmdToEyeViewOffset[2];
  hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeViewOffset;
  hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeViewOffset;

  return std::fabs(hmdToEyeViewOffset[0].x) + std::fabs(hmdToEyeViewOffset[1].x);
}

math::vec2ui OculusWindow::get_eye_resolution() const {
#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
  ovrSizei recommenedTex0Size = ovrHmd_GetFovTextureSize(registered_HMD_, ovrEye_Left,
      registered_HMD_->DefaultEyeFov[0], 1.0f);
#else
  ovrSizei recommenedTex0Size = ovr_GetFovTextureSize(registered_HMD_, ovrEye_Left,
    registered_HMD_desc_.DefaultEyeFov[0], 1.0f);
#endif
  //return math::vec2ui(recommenedTex0Size.w, recommenedTex0Size.h);
  #ifdef _WIN32
    return math::vec2ui(recommenedTex0Size.h, recommenedTex0Size.w);
  #else
    if (has_vertical_display_) {
      return math::vec2ui(recommenedTex0Size.h, recommenedTex0Size.w);
    } else {
      return math::vec2ui(recommenedTex0Size.w, recommenedTex0Size.h);      
    }
  #endif
}

math::vec2ui OculusWindow::get_window_resolution() const {

#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
  ovrSizei HMD_resolution = registered_HMD_->Resolution;
#else
  ovrSizei HMD_resolution = registered_HMD_desc_.Resolution;
#endif

  #ifdef _WIN32
    return math::vec2ui(HMD_resolution.w, HMD_resolution.h);
  #else
    if ( has_vertical_display_) {
      return math::vec2ui(HMD_resolution.h, HMD_resolution.w);
    } else {
      return math::vec2ui(HMD_resolution.w,  HMD_resolution.h);
    }
  #endif
}

// retrieve the oculus sensor orientation for the application
gua::math::mat4 OculusWindow::get_oculus_sensor_orientation() const {
  return oculus_sensor_orientation_;
}



void OculusWindow::start_frame() {
    GlfwWindow::start_frame();

  #ifdef _WIN32

#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
    ovrFrameTiming   ftiming = ovrHmd_GetFrameTiming(registered_HMD_, 0);
    ovrTrackingState hmdState = ovrHmd_GetTrackingState(registered_HMD_, ftiming.DisplayMidpointSeconds);
    swap_texures_->CurrentIndex = (swap_texures_->CurrentIndex + 1) % swap_texures_->TextureCount;

    ovrEyeRenderDesc eyeRenderDesc[2];
    ovrVector3f      hmdToEyeViewOffset[2];
    eyeRenderDesc[0] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_->DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_->DefaultEyeFov[1]);
    hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeViewOffset;
    hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeViewOffset;

    ovr_CalcEyePoses(hmdState.HeadPose.ThePose, hmdToEyeViewOffset, color_layer_.RenderPose);
#else
    auto ftiming = ovr_GetPredictedDisplayTime(registered_HMD_, 0);

    ovrTrackingState hmdState = ovr_GetTrackingState(registered_HMD_, ftiming, true);
    swap_texures_->CurrentIndex = (swap_texures_->CurrentIndex + 1) % swap_texures_->TextureCount;

    ovrEyeRenderDesc eyeRenderDesc[2];
    ovrVector3f      hmdToEyeViewOffset[2];
    eyeRenderDesc[0] = ovr_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_desc_.DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovr_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_desc_.DefaultEyeFov[1]);
    hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeViewOffset;
    hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeViewOffset;

    ovr_CalcEyePoses(hmdState.HeadPose.ThePose, hmdToEyeViewOffset, color_layer_.RenderPose);
#endif
  #else
    ovrEyeRenderDesc eyeRenderDesc[2];
    eyeRenderDesc[0] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_->DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_->DefaultEyeFov[1]);

    ovrFrameTiming   ftiming = ovrHmd_BeginFrameTiming(registered_HMD_, 0);
    ovrTrackingState hmdState = ovrHmd_GetTrackingState(registered_HMD_, ftiming.ScanoutMidpointSeconds);
  #endif

    if (hmdState.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)) {

      auto pose = hmdState.HeadPose.ThePose;

      scm::math::quat<double> rot_quat(pose.Orientation.w,
          pose.Orientation.x,
          pose.Orientation.y,
          pose.Orientation.z);

      oculus_sensor_orientation_ = scm::math::make_translation((double)pose.Position.x, (double)pose.Position.y, (double)pose.Position.z) * rot_quat.to_matrix();
    }
}

void OculusWindow::finish_frame() {
#ifdef _WIN32
    ovrLayerHeader* layers = &color_layer_.Header;

#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
    ovrResult       result = ovrHmd_SubmitFrame(registered_HMD_, 0, nullptr, &layers, 1);
#else
    ovrResult       result = ovr_SubmitFrame(registered_HMD_, 0, nullptr, &layers, 1);
#endif
    if (!OVR_SUCCESS(result)) {
        gua::Logger::LOG_WARNING << "Failed to submit frame to the oculus rift.\n";
    }
#else
    ovrHmd_EndFrameTiming(registered_HMD_);
#endif

    GlfwWindow::finish_frame();
}

void OculusWindow::calculate_viewing_setup() {
  // use just any near and far plane values, since they get reset by guacamole
  float near_distance = 0.5;
  float far_distance = 500.0;
  
  // eye to screen distance can be choosen relatively arbitrary as well, since
  // the virtual screen is enlarged, such that it fits into the frustum at this
  // distance
  float eye_to_screen_distance = 0.08;
  
  // do the viewing setup calculations for both eyes
  for (unsigned eye_num = 0; eye_num < 2; ++eye_num) {
    //retreive the correct projection matrix from the oculus API
#if OVR_PRODUCT_VERSION <= 0 && OVR_MAJOR_VERSION <= 6
    auto const& oculus_eye_projection 
      = ovrMatrix4f_Projection(registered_HMD_->DefaultEyeFov[eye_num], near_distance, far_distance, 1);
#else

    auto const& oculus_eye_projection
      = ovrMatrix4f_Projection(registered_HMD_desc_.DefaultEyeFov[eye_num], near_distance, far_distance, 1);

#endif
    
    //convert the matrix to a gua compatible one
    scm::math::mat4 scm_eye_proj_matrix;
    for( int outer = 0; outer < 4; ++outer ) {
      for( int inner = 0; inner < 4; ++inner ){
        scm_eye_proj_matrix[outer*4 + inner] = oculus_eye_projection.M[outer][inner];
      }
    }

    // unproject one frustum corner defining one clipping plane
    scm::math::mat4 inv_oculus_eye_proj_matrix 
      = scm::math::inverse(scm_eye_proj_matrix);
    scm::math::vec4 back_right_top_frustum_corner 
      = scm::math::vec4(-1.0, -1.0, -1.0,  1.0) * inv_oculus_eye_proj_matrix;
    scm::math::vec4 back_left_bottom_frustum_corner 
      = scm::math::vec4( 1.0,  1.0, -1.0,  1.0) * inv_oculus_eye_proj_matrix;

    scm::math::vec4 normalized_back_left_bottom_frustum_corner
      = back_left_bottom_frustum_corner / back_left_bottom_frustum_corner[3];
    scm::math::vec4 normalized_back_right_top_frustum_corner
      = back_right_top_frustum_corner / back_right_top_frustum_corner[3];

    //note: eye is in (0,0,0 relative to the frustum)

    //solve for t1 along the ray to the upper right corner for our screen distance
    float t_to_screen_along_eye_to_right_upper_corner_vec = ( eye_to_screen_distance ) / normalized_back_right_top_frustum_corner[2];

    //retreive the upper right screen corner
    scm::math::vec4 screen_right_upper_corner_relative_to_eye = scm::math::vec4(t_to_screen_along_eye_to_right_upper_corner_vec * normalized_back_right_top_frustum_corner[0],
                                                                                t_to_screen_along_eye_to_right_upper_corner_vec * normalized_back_right_top_frustum_corner[1],
                                                                                t_to_screen_along_eye_to_right_upper_corner_vec* normalized_back_right_top_frustum_corner[2],
                                                                                1.0  );

    //solve for t2 along the ray to the lower left corner for our screen distance
    float t_to_screen_along_eye_to_left_lower_corner_vec = ( eye_to_screen_distance ) / normalized_back_left_bottom_frustum_corner[2];

    //retreive the lower left screen corner
    scm::math::vec4 screen_left_lower_corner_relative_to_eye = scm::math::vec4( t_to_screen_along_eye_to_left_lower_corner_vec * normalized_back_left_bottom_frustum_corner[0],
                                                                                t_to_screen_along_eye_to_left_lower_corner_vec * normalized_back_left_bottom_frustum_corner[1],
                                                                                t_to_screen_along_eye_to_left_lower_corner_vec* normalized_back_left_bottom_frustum_corner[2],
                                                                                1.0  );

    // calculate the horizontal and vertical additional offsets of the eye based on the position of the
    // screen corners in relation to the eye
    float horizontal_eye_distortion_offset =  (  std::fabs(screen_left_lower_corner_relative_to_eye[0])
                                                - std::fabs(screen_right_upper_corner_relative_to_eye[0]))/2.0;

    float vertical_eye_distortion_offset =  (  std::fabs(screen_left_lower_corner_relative_to_eye[1])
                                                - std::fabs(screen_right_upper_corner_relative_to_eye[1]))/2.0;

    // get default horizontal offset for both eyes
    float half_IPD = get_IPD() / 2.0;

    // choose the correct initial eye offste
    float standard_horizontal_eye_offset = eye_num == 0 ? -half_IPD : half_IPD;

    // calculate the final screen translation
    screen_translation_[eye_num] 
      = gua::math::vec3(standard_horizontal_eye_offset + horizontal_eye_distortion_offset,
                        0.0 + vertical_eye_distortion_offset,
                        -eye_to_screen_distance);

    // calculate the final screen size
    screen_size_[eye_num] 
      = gua::math::vec2f( (screen_right_upper_corner_relative_to_eye 
                                             - screen_left_lower_corner_relative_to_eye) );
  }

}

} //namespace gua
