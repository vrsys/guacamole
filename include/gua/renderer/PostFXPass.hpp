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

#ifndef GUA_POST_FX_PASS_HPP
#define GUA_POST_FX_PASS_HPP

// guacamole headers
#include <gua/renderer/FullscreenPass.hpp>
#include <gua/renderer/BuiltInTextures.hpp>

namespace gua {

class PostGBufferMeshUberShader;
class GBuffer;

/**
 *
 */
class PostFXPass : public Pass {
 public:

  /**
   *
   */
  PostFXPass(Pipeline* pipeline);

  /**
   * Destructor.
   *
   * Deletes the FullscreenPass and frees all associated data.
   */
  virtual ~PostFXPass();

  void create(
      RenderContext const& ctx,
      std::vector<std::pair<BufferComponent,
                            scm::gl::sampler_state_desc> > const& layers);

  void render_scene(Camera const& camera, RenderContext const& ctx);

  /* virtual */ LayerMapping const* get_gbuffer_mapping() const;

  void print_shaders(std::string const& directory,
                     std::string const& name) const;

  bool pre_compile_shaders(RenderContext const& ctx);

 private:
  void pre_rendering(Camera const& camera,
                     SerializedScene const& scene,
                     CameraMode eye,
                     RenderContext const& ctx);

  void render_previews(CameraMode eye, RenderContext const& ctx);
  void render_fxaa(RenderContext const& ctx);
  void render_fog(RenderContext const& ctx);
  void render_vignette(RenderContext const& ctx);
  void render_glow(CameraMode eye, RenderContext const& ctx);
  bool render_godrays(Camera const& camera,
                      SerializedScene const& scene,
                      CameraMode eye,
                      RenderContext const& ctx);
  void render_ssao(RenderContext const& ctx);
  void render_hdr(RenderContext const& ctx, std::shared_ptr<Texture2D> const& texture);

  // postfx_shaders 0:  SSAO, Fog, God Rays,
  // postfx_shaders 1:  Glow,
  // postfx_shaders 2:  HDR,
  // postfx_shaders 3:  FXAA, Vignette
  std::vector<ShaderProgram*> postfx_shaders_;
  StereoBuffer* ping_buffer_;
  StereoBuffer* pong_buffer_;

  TextRenderer* fps_text_renderer_;
  TextRenderer* preview_text_renderer_;

  ShaderProgram* god_ray_shader_;
  ShaderProgram* fullscreen_texture_shader_;
  ShaderProgram* glow_shader_;
  ShaderProgram* luminance_shader_;

  std::vector<GBuffer*> godray_buffers_;
  std::vector<GBuffer*> glow_buffers_;
  GBuffer* luminance_buffer_;
  float last_frame_luminance_;

  scm::gl::quad_geometry_ptr fullscreen_quad_;
  scm::gl::depth_stencil_state_ptr depth_stencil_state_;
  scm::gl::blend_state_ptr blend_add_;

  NoiseTexture noise_texture_;
};

}

#endif  // GUA_POST_FX_PASS_HPP
