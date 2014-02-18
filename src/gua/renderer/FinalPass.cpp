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
#include <gua/renderer/FinalPass.hpp>

// guacamole headers
#include <gua/renderer/FinalUberShader.hpp>
#include <gua/databases.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/utils.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

FinalPass::FinalPass(Pipeline* pipeline)
    : FullscreenPass(pipeline), shader_(new FinalUberShader) {}

////////////////////////////////////////////////////////////////////////////////

void FinalPass::print_shaders(std::string const& directory,
                              std::string const& name) const {
  shader_->save_to_file(directory, name + "/final");
}

////////////////////////////////////////////////////////////////////////////////

bool FinalPass::pre_compile_shaders(RenderContext const& ctx) {
    if (shader_) return shader_->upload_to(ctx);
    return false;
}

////////////////////////////////////////////////////////////////////////////////

void FinalPass::apply_material_mapping(
    std::set<std::string> const& materials,
    std::vector<LayerMapping const*> const& inputs) const {
  shader_->create(materials, inputs);
}

////////////////////////////////////////////////////////////////////////////////

void FinalPass::set_uniforms(SerializedScene const& scene,
                             RenderContext const& ctx) {

  shader_->set_material_uniforms(
      scene.materials_, ShadingModel::FINAL_STAGE, ctx);

  shader_->set_uniform(
      ctx, pipeline_->config.ambient_color(), "gua_ambient_color");
  shader_->set_uniform(ctx,
                       static_cast<int>(pipeline_->config.background_mode()),
                       "gua_background_mode");

  if (pipeline_->config.background_mode() == BackgroundMode::COLOR || pipeline_->config.background_texture() == "")
    shader_->set_uniform(
        ctx, pipeline_->config.background_color(), "gua_background_color");
  else
    shader_->set_uniform(ctx,
                         TextureDatabase::instance()->lookup(
                             pipeline_->config.background_texture()),
                         "gua_background_texture");
}

////////////////////////////////////////////////////////////////////////////////

LayerMapping const* FinalPass::get_gbuffer_mapping() const {
  return shader_->get_gbuffer_mapping();
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void FinalPass::rendering(Camera const& camera,
                                        SerializedScene const& scene,
                                        CameraMode eye,
                                        RenderContext const& ctx) {

  Pass::bind_inputs(*shader_, eye, ctx);
  Pass::set_camera_matrices(
      *shader_, camera, pipeline_->get_current_scene(eye), eye, ctx);

  shader_->use(ctx);
  fullscreen_quad_->draw(ctx.render_context);
  shader_->unuse(ctx);
}

}
