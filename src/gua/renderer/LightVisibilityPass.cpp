// class header
#include <gua/renderer/LightVisibilityPass.hpp>

#include <gua/renderer/LightVisibilityRenderer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>

#include <scm/gl_core/render_device/opengl/gl_core.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

LightVisibilityPassDescription::LightVisibilityPassDescription()
  : PipelinePassDescription() {
  vertex_shader_ = "shaders/light_visibility.vert";
  fragment_shader_ = "shaders/light_visibility.frag";
  needs_color_buffer_as_input_ = false; // don't ping pong the color buffer
  writes_only_color_buffer_ = false; // we don't write out a color
  doClear_ = false;
  rendermode_ = RenderMode::Custom;

  depth_stencil_state_ = boost::make_optional(
      scm::gl::depth_stencil_state_desc(false, false));

  rasterizer_state_ = boost::make_optional(scm::gl::rasterizer_state_desc(
        scm::gl::FILL_SOLID, scm::gl::CULL_FRONT));
}

////////////////////////////////////////////////////////////////////////////////

PipelinePassDescription* LightVisibilityPassDescription::make_copy() const {
  return new LightVisibilityPassDescription(*this);
}

////////////////////////////////////////////////////////////////////////////////

PipelinePass LightVisibilityPassDescription::make_pass(RenderContext const& ctx,
                                                       SubstitutionMap& substitution_map)
{
  auto const& glapi = ctx.render_context->opengl_api();
  substitution_map["light_table_tile_power"] = std::to_string(tile_power_);
  substitution_map["light_table_fallback_mode"] =
      (rasterization_mode() == FULLSCREEN_FALLBACK) ? "1" : "0";

  if (    rasterization_mode() == CONSERVATIVE
      && !glapi.extension_NV_conservative_raster) {
    Logger::LOG_WARNING << "CONSERVATIVE rasterization mode is not supported on this hardware. "
                        << "Falling back to SIMPLE mode." << std::endl;
    rasterization_mode(SIMPLE);
  }

  int tp = tile_power_;
  unsigned ms_sample_count{};
  bool enable_conservative = false;
  bool enable_fullscreen_fallback = false;

  // determine rasterization properties
  switch (rasterization_mode()) {
    case AUTO:
      if (glapi.extension_NV_conservative_raster) {
        enable_conservative = true; }
      else { ms_sample_count = 4; }
      break;
    case SIMPLE: /* nothing */ break;
    case CONSERVATIVE:
      enable_conservative = true; break;
    case MULTISAMPLED_2:
      ms_sample_count = 2; break;
    case MULTISAMPLED_4:
      ms_sample_count = 4; break;
    case MULTISAMPLED_8:
      ms_sample_count = 8; break;
    case MULTISAMPLED_16:
      ms_sample_count = 16; break;
    case FULLSCREEN_FALLBACK:
      enable_fullscreen_fallback = true; break;
  };

  // use traditional rasterization if tile size == pixel size
  if (tp == 0) {
    enable_conservative = false;
    ms_sample_count = 0;
  }

  PipelinePass pass{*this, ctx, substitution_map};

  auto renderer = std::make_shared<LightVisibilityRenderer>();

  pass.process_ = [renderer, tp, ms_sample_count, enable_conservative, enable_fullscreen_fallback](
    PipelinePass& pass, PipelinePassDescription const& desc, Pipeline & pipe) {
    pipe.get_context().render_context->set_depth_stencil_state(pass.depth_stencil_state_);
    pipe.get_context().render_context->set_rasterizer_state(pass.rasterizer_state_);
    renderer->render(pass, pipe, tp, ms_sample_count, enable_conservative, enable_fullscreen_fallback);
  };

  return pass;
}

////////////////////////////////////////////////////////////////////////////////

}
