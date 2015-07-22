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

// external headers
#include <iostream>


namespace gua {

void OculusSDK2Window::initialize_distortion_meshes(ovrHmd const& hmd, RenderContext const& ctx) {
  ovrEyeRenderDesc eyeRenderDesc[2];

  eyeRenderDesc[0] = ovrHmd_GetRenderDesc( hmd, ovrEye_Left, hmd->DefaultEyeFov[0] );
  eyeRenderDesc[1] = ovrHmd_GetRenderDesc( hmd, ovrEye_Right, hmd->DefaultEyeFov[1] );

  // Create the Distortion Meshes:

  OculusSDK2DistortionMesh distortion_mesh_cpu_buffer[2];




  unsigned index_buffer_offset = 0;
  for ( int eyeNum = 0; eyeNum < 2; eyeNum ++ )
  {
    ovrDistortionMesh meshData;
    ovrHmd_CreateDistortionMesh( hmd,
        eyeRenderDesc[eyeNum].Eye,
        eyeRenderDesc[eyeNum].Fov,
        0,
        &meshData );



    OVR::Sizei eyeTextureSize = OVR::Sizei(1920/2, 1080);

    ovrVector2f UVScaleOffset[2];
    ovrRecti viewports[2];
    viewports[0].Pos.x = 0;
    viewports[0].Pos.y = 0;
    viewports[0].Size.w = eyeTextureSize.w;
    viewports[0].Size.h = eyeTextureSize.h;
    viewports[1].Pos.x = eyeTextureSize.w;
    viewports[1].Pos.y = 0;
    viewports[1].Size.w = eyeTextureSize.w;
    viewports[1].Size.h = eyeTextureSize.h;

    if( eyeNum == 0 )
    {
      ovrHmd_GetRenderScaleAndOffset( eyeRenderDesc[eyeNum].Fov,
          eyeTextureSize, viewports[eyeNum],
          UVScaleOffset);
     // params = mMatLeft->getTechnique(0)->getPass(0)->getVertexProgramParameters();
    } else {
      ovrHmd_GetRenderScaleAndOffset( eyeRenderDesc[eyeNum].Fov,
          eyeTextureSize, viewports[eyeNum],
          UVScaleOffset);
     // params = mMatRight->getTechnique(0)->getPass(0)->getVer texProgramParameters();
    }

    bool isLeftEye = 0 == eyeNum;

    distortion_mesh_cpu_buffer[eyeNum].add_distortion_mesh_component(meshData, UVScaleOffset, isLeftEye);

//TODO: SHADER UPLOAD HERE!
/*
    params->setNamedConstant( "eyeToSourceUVScale",
        Ogre::Vector2( UVScaleOffset[0].x, UVScaleOffset[0].y ) );
    params->setNamedConstant( "eyeToSourceUVOffset",
        Ogre::Vector2( UVScaleOffset[1].x, UVScaleOffset[1].y ) );
*/
    std::cout << "UVScaleOffset[0]: " << UVScaleOffset[0].x << ", " << UVScaleOffset[0].y << std::endl;
    std::cout << "UVScaleOffset[1]: " << UVScaleOffset[1].x << ", " << UVScaleOffset[1].y << std::endl;



    std::cout << "Distortion Mesh " << eyeNum << " :\n";
    for( unsigned int i = 0; i < meshData.VertexCount; i++ )
    {
      //ovrDistortionVertex v = meshData.pVertexData[i];
      //std::cout << (UVScaleOffset[0].x*v.TanEyeAnglesR.x + UVScaleOffset[1].x) / 2.0  
      //          << " " << UVScaleOffset[0].y*v.TanEyeAnglesR.y + UVScaleOffset[1].y << " " << 0.0 << "\n";

/*
      manual->position( v.ScreenPosNDC.x,
          v.ScreenPosNDC.y, 0 );
      manual->textureCoord( v.TanEyeAnglesR.x,//*UVScaleOffset[0].x + UVScaleOffset[1].x,
          v.TanEyeAnglesR.y);//*UVScaleOffset[0].y + UVScaleOffset[1].y);
      manual->textureCoord( v.TanEyeAnglesG.x,//*UVScaleOffset[0].x + UVScaleOffset[1].x,
          v.TanEyeAnglesG.y);//*UVScaleOffset[0].y + UVScaleOffset[1].y);
      manual->textureCoord( v.TanEyeAnglesB.x,//*UVScaleOffset[0].x + UVScaleOffset[1].x,
          v.TanEyeAnglesB.y);//*UVScaleOffset[0].y + UVScaleOffset[1].y);
      float vig = std::max( v.VignetteFactor, (float)0.0 );
      manual->colour( vig, vig, vig, vig );
*/
    }


    ovrHmd_DestroyDistortionMesh( &meshData );


  std::cout << "Before first accessing context.\n";

  //create gpu buffer for distortion mesh vertices
  std::cout << "Num vertices: " << distortion_mesh_cpu_buffer[eyeNum].num_vertices << "\n";

  distortion_mesh_vertices_[eyeNum] = 
    ctx.render_device
      ->create_buffer(scm::gl::BIND_VERTEX_BUFFER,
                      scm::gl::USAGE_STATIC_DRAW,
                      distortion_mesh_cpu_buffer[eyeNum].num_vertices
                        * sizeof(OculusSDK2DistortionMesh::DistortionVertex),
                      0);
  std::cout << "After first accessing context.\n";

  //map it to cpu
  OculusSDK2DistortionMesh::DistortionVertex* 
  data(static_cast<OculusSDK2DistortionMesh::DistortionVertex*>
    (ctx.render_context->map_buffer(distortion_mesh_vertices_[eyeNum],
                      scm::gl::ACCESS_WRITE_INVALIDATE_BUFFER)));

  //fill it with the pre-loaded cpu data
  distortion_mesh_cpu_buffer[eyeNum].copy_to_buffer(data);

  //unmap it
  ctx.render_context->unmap_buffer(distortion_mesh_vertices_[eyeNum]);


  distortion_mesh_indices_[eyeNum] =
    ctx.render_device
      ->create_buffer(scm::gl::BIND_INDEX_BUFFER,
                      scm::gl::USAGE_STATIC_DRAW,
                      distortion_mesh_cpu_buffer[eyeNum].num_indices
                        * sizeof(unsigned),
                      &(distortion_mesh_cpu_buffer[eyeNum].indices[0]) );

  distortion_mesh_vertex_array_[eyeNum] = 
    ctx.render_device->create_vertex_array(
                       distortion_mesh_cpu_buffer[eyeNum].get_vertex_format(),
                       {distortion_mesh_vertices_[eyeNum]});

  ctx.render_context->apply_vertex_input();

  num_distortion_mesh_indices[eyeNum] = distortion_mesh_cpu_buffer[eyeNum].num_indices;



  }



  std::cout << "After doing everything.\n";
}

OculusSDK2Window::OculusSDK2Window(std::string const& display, ovrHmd const& hmd):
  Window(),
  num_distortion_mesh_indices({0,0}),
  distortion_(4) {

  config.set_size(math::vec2ui(1920, 1080));
  config.set_title("guacamole");
  config.set_display_name(display);
  config.set_stereo_mode(StereoMode::SIDE_BY_SIDE);
  config.set_left_resolution(math::vec2ui(1920/2, 1080));
  config.set_left_position(math::vec2ui(0, 0));
  config.set_right_resolution(math::vec2ui(1920/2, 1080));
  config.set_right_position(math::vec2ui(1920/2, 0));

  // for now fixed distortion values TODO should be set dynamically by OVR
  set_distortion(1.0, 0.0, 0.0, 0.0);

  registeredHMD = hmd;
}

////////////////////////////////////////////////////////////////////////////////

OculusSDK2Window::~OculusSDK2Window() {

}

////////////////////////////////////////////////////////////////////////////////

void OculusSDK2Window::init_context() {
/*
  fullscreen_shader_.create_from_sources(R"(
      #version 420
      #extension GL_NV_bindless_texture : require

      layout(location=0) in vec3 in_position;

      out vec2 tex_coord;

      void main() {
          tex_coord = in_position.xy*0.5 + 0.5;
          gl_Position = vec4(in_position, 1.0);
      }
    )", R"(
      #version 420
      #extension GL_NV_bindless_texture : require
      #extension GL_NV_gpu_shader5      : enable

      in vec2 tex_coord;

      uniform uvec2 sampler;

      // oculus parameters
      uniform vec2 lens_center;
      uniform vec2 scale;
      uniform vec4 hmd_warp_param;

      layout (location = 0) out vec3 out_color;

      sampler2D get_tex(uvec2 handle) {
          return sampler2D(uint64_t(handle.x) + uint64_t(handle.y) * 4294967295);
      }

      vec2 hmd_warp(vec2 in_texcoord) {
          vec2 theta = (in_texcoord - lens_center) * 2.0; // Scales to [-1, 1]
          float rSq = theta.x * theta.x + theta.y * theta.y;
          vec2 rvector = theta * (hmd_warp_param.x+hmd_warp_param.y*rSq
                                                  +hmd_warp_param.z*rSq*rSq
                                                  +hmd_warp_param.w*rSq*rSq*rSq);
          return lens_center + scale * rvector;
      }

      vec3 get_color() {

          vec2 tc = hmd_warp(tex_coord);

          if (tc.x < 0.0 || tc.y < 0.0 || tc.x > 1.0 || tc.y > 1.0 )
              return vec3(0);

          return vec3(texture2D( get_tex(sampler), tc).rgb);
      }

      void main() {
          out_color = get_color();
      }
    )");
*/

  fullscreen_shader_.create_from_sources(R"(
      #version 420
      #extension GL_NV_bindless_texture : require

      layout(location=0) in vec2 in_position;
      layout(location=1) in vec2 in_tex_coords_r;
      layout(location=2) in vec2 in_tex_coords_g;
      layout(location=3) in vec2 in_tex_coords_b;

      out vec2 tex_coord;

      void main() {
          //tex_coord = in_position.xy*0.5 + 0.5;
          gl_Position = vec4(in_position, 0.0, 1.0);
      }
    )", R"(
      #version 420
      #extension GL_NV_bindless_texture : require
      #extension GL_NV_gpu_shader5      : enable

      //in vec2 tex_coord;

      uniform uvec2 sampler;

      // oculus parameters
      uniform vec2 lens_center;
      uniform vec2 scale;
      uniform vec4 hmd_warp_param;

      layout (location = 0) out vec3 out_color;

      sampler2D get_tex(uvec2 handle) {
          return sampler2D(uint64_t(handle.x) + uint64_t(handle.y) * 4294967295);
      }

      vec2 hmd_warp(vec2 in_texcoord) {
          vec2 theta = (in_texcoord - lens_center) * 2.0; // Scales to [-1, 1]
          float rSq = theta.x * theta.x + theta.y * theta.y;
          vec2 rvector = theta * (hmd_warp_param.x+hmd_warp_param.y*rSq
                                                  +hmd_warp_param.z*rSq*rSq
                                                  +hmd_warp_param.w*rSq*rSq*rSq);
          return lens_center + scale * rvector;
      }

      vec3 get_color() {

          //vec2 tc = hmd_warp(tex_coord);
          vec2 tc = vec2(1.0, 1.0);
          if (tc.x < 0.0 || tc.y < 0.0 || tc.x > 1.0 || tc.y > 1.0 )
              return vec3(0);

          return vec3(texture2D( get_tex(sampler), tc).rgb);
      }

      void main() {
          out_color = get_color();
      }
    )");

  ctx_.render_device  = scm::gl::render_device_ptr(new scm::gl::render_device());
  ctx_.render_context = ctx_.render_device->main_context();

  {
    std::lock_guard<std::mutex> lock(last_context_id_mutex_);
    ctx_.id = last_context_id_++;
  }

  ctx_.render_window = this;

  fullscreen_quad_ = scm::gl::quad_geometry_ptr(new scm::gl::quad_geometry(
    ctx_.render_device, scm::math::vec2f(-1.f, -1.f), scm::math::vec2f(1.f, 1.f)));

  initialize_distortion_meshes(registeredHMD, *get_context());

  depth_stencil_state_ = ctx_.render_device
      ->create_depth_stencil_state(false, false, scm::gl::COMPARISON_NEVER);

  blend_state_ = ctx_.render_device->create_blend_state(true,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE,
                                                        scm::gl::FUNC_ONE);
  if (config.get_debug()) {
    ctx_.render_context->register_debug_callback(boost::make_shared<DebugOutput>());
  }


}

////////////////////////////////////////////////////////////////////////////////

void OculusSDK2Window::set_distortion(math::vec4 const& distortion) {
  distortion_ = distortion;
}

////////////////////////////////////////////////////////////////////////////////

void OculusSDK2Window::set_distortion(float distortion0, float distortion1, float distortion2, float distortion3) {
  distortion_[0] = distortion0;
  distortion_[1] = distortion1;
  distortion_[2] = distortion2;
  distortion_[3] = distortion3;
}

////////////////////////////////////////////////////////////////////////////////

void OculusSDK2Window::display(std::shared_ptr<Texture> const& texture, bool is_left) {

  fullscreen_shader_.use(*get_context());
  fullscreen_shader_.set_uniform(*get_context(), texture->get_handle(ctx_), "sampler");

  if (is_left)
    fullscreen_shader_.set_uniform(*get_context(), math::vec2f(0.6f, 0.5f), "lens_center");
  else
    fullscreen_shader_.set_uniform(*get_context(), math::vec2f(0.4f, 0.5f), "lens_center");

  fullscreen_shader_.set_uniform(*get_context(), math::vec2f(0.4f, 0.4f), "scale");
  fullscreen_shader_.set_uniform(*get_context(), math::vec4f(distortion_), "hmd_warp_param");

  if (is_left)
    get_context()->render_context->set_viewport(scm::gl::viewport(config.get_left_position(), config.get_left_resolution()));
  else
    get_context()->render_context->set_viewport(scm::gl::viewport(config.get_right_position(), config.get_right_resolution()));
    get_context()->render_context->set_depth_stencil_state(depth_stencil_state_);

    get_context()->render_context->set_rasterizer_state(

          get_context()->render_device->create_rasterizer_state(scm::gl::FILL_SOLID, scm::gl::CULL_NONE, scm::gl::ORIENT_CCW, true)

      );

  //get_context()->render_context->apply();

  //fullscreen_quad_->draw(get_context()->render_context);


  unsigned current_eye_num = is_left ? 0 : 1;

  get_context()->render_context->bind_vertex_array(distortion_mesh_vertex_array_[current_eye_num]);
  get_context()->render_context->bind_index_buffer(distortion_mesh_indices_[current_eye_num], 
    scm::gl::PRIMITIVE_TRIANGLE_LIST, scm::gl::TYPE_UINT);

  get_context()->render_context->apply();

  //get_context()->render_context->draw_elements(3);

  
  get_context()->render_context->draw_elements(num_distortion_mesh_indices[current_eye_num]);
  std::cout << "Num distortion mesh indices: " << num_distortion_mesh_indices[current_eye_num] << "\n";
  get_context()->render_context->reset_state_objects();
  
  fullscreen_shader_.unuse(*get_context());

}

}
