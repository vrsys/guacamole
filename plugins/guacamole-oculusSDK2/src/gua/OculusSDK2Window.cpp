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
#include <gua/OculusSDK2Window.hpp>

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
bool OculusSDK2Window::oculus_environment_initialized_ = false;
unsigned OculusSDK2Window::registered_oculus_device_count_ = 0;

// ---------------------------------------------------------------
// static functions
// ---------------------------------------------------------------
// calling this once in the application before any window creation
// will set up the oculus environment
void OculusSDK2Window::initialize_oculus_environment() {
  if( !oculus_environment_initialized_ ) {
      ovr_Initialize(nullptr);
      oculus_environment_initialized_ = true;
  } else {
      Logger::LOG_WARNING << "Attempt to initialize oculus environment multiple times!"
                          << std::endl;
  }
}

// calling this once in the application after the windows are destroyed will shutdown
// the oculus environment
void OculusSDK2Window::shutdown_oculus_environment() {
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
void OculusSDK2Window::initialize_distortion_meshes(ovrHmd const& hmd, RenderContext const& ctx) {
  ovrEyeRenderDesc eyeRenderDesc[2];

  eyeRenderDesc[0] = ovrHmd_GetRenderDesc( hmd, ovrEye_Left, hmd->DefaultEyeFov[0] );
  eyeRenderDesc[1] = ovrHmd_GetRenderDesc( hmd, ovrEye_Right, hmd->DefaultEyeFov[1] );

  // Create the Distortion Meshes (one for each eye):
  OculusSDK2DistortionMesh distortion_mesh_cpu_buffer[2];


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
                      * sizeof(OculusSDK2DistortionMesh::DistortionVertex),
                    0);

  // map it to cpu
  OculusSDK2DistortionMesh::DistortionVertex*
  data(static_cast<OculusSDK2DistortionMesh::DistortionVertex*>
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


OculusSDK2Window::OculusSDK2Window(std::string const& display):
  GlfwWindow() {

  // automatically register the HMDs in order
#ifdef _WIN32
    ovrHmd_Create(registered_oculus_device_count_++, &registered_HMD_);
#else
    registered_HMD_ = ovrHmd_Create(registered_oculus_device_count_++);
#endif

  // register all three sensors for sensor fusion
  bool tracking_configured = ovrHmd_ConfigureTracking(registered_HMD_, ovrTrackingCap_Orientation
      | ovrTrackingCap_MagYawCorrection
      | ovrTrackingCap_Position, 0);

  std::cout << registered_HMD_->ProductName << "\n";

  if (!tracking_configured) {
      ovrHmd_Destroy(registered_HMD_);
      registered_HMD_ = NULL;
      gua::Logger::LOG_WARNING << "The oculus device does not support demanded tracking feature.\n";

      exit(-1);
  }

  // set up the window parameters
#ifdef _WIN32
  ovrSizei recommenedTex0Size = ovrHmd_GetFovTextureSize(registered_HMD_, ovrEye_Left,
      registered_HMD_->DefaultEyeFov[0], 1.0f);
  ovrSizei recommenedTex1Size = ovrHmd_GetFovTextureSize(registered_HMD_, ovrEye_Right,
      registered_HMD_->DefaultEyeFov[1], 1.0f);
  ovrSizei bufferSize;
  bufferSize.w = recommenedTex0Size.w + recommenedTex1Size.w;
  bufferSize.h = std::max(recommenedTex0Size.h, recommenedTex1Size.h);

  config.set_size(math::vec2ui(bufferSize.w, bufferSize.h));
  config.set_left_resolution(math::vec2ui(recommenedTex0Size.w, recommenedTex0Size.h));
  config.set_left_position(math::vec2ui(0, 0));
  config.set_right_resolution(math::vec2ui(recommenedTex1Size.w, recommenedTex1Size.h));
  config.set_right_position(math::vec2ui(recommenedTex0Size.w, 0));

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
  color_layer_.ColorTexture[0] = swap_texures_;
  color_layer_.ColorTexture[1] = swap_texures_;
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

#else
  // get the resolution from the device itself
  unsigned res_x = registered_HMD_->Resolution.w;
  unsigned res_y = registered_HMD_->Resolution.h;

  config.set_size(math::vec2ui(res_x, res_y));
  config.set_left_resolution(math::vec2ui(res_x / 2, res_y));
  config.set_left_position(math::vec2ui(0, 0));
  config.set_right_resolution(math::vec2ui(res_x / 2, res_y));
  config.set_right_position(math::vec2ui(res_x / 2, 0));
#endif

  config.set_title("guacamole");
  config.set_display_name(display);
  config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);
  
}

////////////////////////////////////////////////////////////////////////////////

OculusSDK2Window::~OculusSDK2Window() {
  // cleanup the oculus device
  ovrHmd_Destroy(registered_HMD_);
}

////////////////////////////////////////////////////////////////////////////////

void OculusSDK2Window::init_context() {

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

#ifdef _WIN32
  
  auto const& glapi = ctx_.render_context->opengl_api();
  glapi.glGenFramebuffers(1, &blit_fbo_read_);
  glapi.glGenFramebuffers(1, &blit_fbo_write_);

  if (ovrHmd_CreateSwapTextureSetGL(registered_HMD_, scm::gl::FORMAT_RGB_32F, config.size().x, config.size().y, &swap_texures_) != ovrSuccess) {
      gua::Logger::LOG_WARNING << "Failed to create swap textures for oculus rift.\n";
  }

#else

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

void OculusSDK2Window::display(std::shared_ptr<Texture> const& texture, bool is_left) {

    unsigned current_eye_num = is_left ? 0 : 1;

#ifdef _WIN32

    auto const& glapi = ctx_.render_context->opengl_api();
    
    ovrGLTexture* tex = (ovrGLTexture*)&swap_texures_->Textures[current_eye_num];


    // setup draw buffer
    glapi.glBindTexture(GL_TEXTURE_2D, tex->OGL.TexId);
    glapi.glBindFramebuffer(GL_DRAW_FRAMEBUFFER, blit_fbo_write_);
    glapi.glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex->OGL.TexId, 0);
    glapi.glFramebufferRenderbuffer(GL_DRAW_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    
    GLenum status = glapi.glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        gua::Logger::LOG_WARNING << "Incomplete.\n";
    }
    
    // setup read buffer
    texture->bind(ctx_, 32);
    glapi.glBindFramebuffer(GL_READ_FRAMEBUFFER, blit_fbo_read_);
    glapi.glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 32, 0);
    glapi.glFramebufferRenderbuffer(GL_READ_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, 0);
    
    status = glapi.glCheckFramebufferStatus(GL_READ_FRAMEBUFFER);
    if (status != GL_FRAMEBUFFER_COMPLETE) {
        gua::Logger::LOG_WARNING << "Incomplete.\n";
    }
    
    glapi.glBlitFramebuffer(0, texture->height(), texture->width(), 0,
        0, 0, texture->width(), texture->height(),
        GL_COLOR_BUFFER_BIT, GL_NEAREST);
    

#else
  auto frameTiming = ovrHmd_BeginFrameTiming(registered_HMD_,0);  

  // use the distortion mesh shader as "fullscreen" shader
  fullscreen_shader_.use(*get_context());
  // upload the usual quad texture to be mapped on the distortion mesh
  fullscreen_shader_.set_uniform(*get_context(), texture->get_handle(ctx_), "sampler");

  if (is_left)
    get_context()->render_context->set_viewport(scm::gl::viewport(config.get_left_position(), config.get_left_resolution()));
  else
    get_context()->render_context->set_viewport(scm::gl::viewport(config.get_right_position(), config.get_right_resolution()));

    get_context()->render_context->set_depth_stencil_state(depth_stencil_state_);

    get_context()->render_context->set_rasterizer_state(
      no_backface_culling_state_
    );

  

  // bind vertex array for the currently rendered eye
  get_context()->render_context->bind_vertex_array(distortion_mesh_vertex_array_[current_eye_num]);
  get_context()->render_context->bind_index_buffer(distortion_mesh_indices_[current_eye_num],
    scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  get_context()->render_context->apply();

  // actually draw the distortion mesh
  get_context()->render_context->draw_elements(num_distortion_mesh_indices_[current_eye_num]);

  get_context()->render_context->reset_state_objects();

  fullscreen_shader_.unuse(*get_context());

  // end frame timing (see above)
  ovrHmd_EndFrameTiming(registered_HMD_);
    
#endif 
}

// retrieve the oculus sensor orientation for the application
gua::math::mat4 OculusSDK2Window::get_oculus_sensor_orientation() const {
    return oculus_sensor_orientation_;
}

void OculusSDK2Window::start_frame() {
  #ifdef _WIN32
    
    // Get both eye poses simultaneously, with IPD offset already included.
    ovrFrameTiming   ftiming = ovrHmd_GetFrameTiming(registered_HMD_, 0);
    ovrTrackingState hmdState = ovrHmd_GetTrackingState(registered_HMD_, ftiming.DisplayMidpointSeconds);

    ovrEyeRenderDesc eyeRenderDesc[2];
    ovrVector3f      hmdToEyeViewOffset[2];
    eyeRenderDesc[0] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Left, registered_HMD_->DefaultEyeFov[0]);
    eyeRenderDesc[1] = ovrHmd_GetRenderDesc(registered_HMD_, ovrEye_Right, registered_HMD_->DefaultEyeFov[1]);
    hmdToEyeViewOffset[0] = eyeRenderDesc[0].HmdToEyeViewOffset;
    hmdToEyeViewOffset[1] = eyeRenderDesc[1].HmdToEyeViewOffset;

    ovr_CalcEyePoses(hmdState.HeadPose.ThePose, hmdToEyeViewOffset, color_layer_.RenderPose);

    if (hmdState.StatusFlags & (ovrStatus_OrientationTracked | ovrStatus_PositionTracked)) {
        // The cpp compatibility layer is used to convert ovrPosef to Posef (see OVR_Math.h)
        auto pose = hmdState.HeadPose.ThePose;

        scm::math::quat<double> rot_quat(pose.Orientation.w,
            pose.Orientation.x,
            pose.Orientation.y,
            pose.Orientation.z);

        oculus_sensor_orientation_ = scm::math::make_translation((double)pose.Position.x, (double)pose.Position.y, (double)pose.Position.z) * rot_quat.to_matrix();
    }

    swap_texures_->CurrentIndex = (swap_texures_->CurrentIndex + 1) % swap_texures_->TextureCount;
  #endif
}

void OculusSDK2Window::finish_frame() {
#ifdef _WIN32
    ovrLayerHeader* layers = &color_layer_.Header;
    ovrResult       result = ovrHmd_SubmitFrame(registered_HMD_, 0, nullptr, &layers, 1);

    if (result != ovrSuccess) {
        gua::Logger::LOG_WARNING << "Failed to submit frame to the oculus rift.\n";
    }
#endif
}

}
