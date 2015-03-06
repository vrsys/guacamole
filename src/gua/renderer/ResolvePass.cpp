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
#include <gua/renderer/ResolvePass.hpp>

#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ABuffer.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/Resources.hpp>
#include <gua/utils/Logger.hpp>

#include <boost/variant.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription::ResolvePassDescription()
  : PipelinePassDescription()
{
  vertex_shader_ = "shaders/common/fullscreen_quad.vert";
  fragment_shader_ = "shaders/resolve.frag";
  name_ = "ResolvePass";
  needs_color_buffer_as_input_ = true;
  writes_only_color_buffer_ = true;
  rendermode_ = RenderMode::Quad;
  depth_stencil_state_ = boost::make_optional(
    scm::gl::depth_stencil_state_desc(false, false));

  // default background configuration
  uniforms["gua_background_mode"] = static_cast<int>(BackgroundMode::COLOR);
  uniforms["gua_background_color"] = scm::math::vec3f(0.2f, 0.2f, 0.2f);
  uniforms["gua_background_texture"] = std::string("gua_default_texture");
  
  // default ambient lighting
  uniforms["gua_environment_lighting_mode"] = static_cast<int>(EnvironmentLightingMode::AMBIENT_COLOR);
  uniforms["gua_environment_lighting_color"] = scm::math::vec3f(0.2f, 0.2f, 0.2f);
  uniforms["gua_environment_lighting_spheremap"] = std::string("gua_default_texture");
  uniforms["gua_environment_lighting_cubemap"] = std::string("gua_default_texture");

  // default ssao
  uniforms["gua_ssao_enable"] = false;
  uniforms["gua_ssao_radius"] = 1.0f;
  uniforms["gua_ssao_intensity"] = 1.0f;
  uniforms["gua_ssao_falloff"] = 0.1f;
  uniforms["gua_noise_tex"] = std::string("gua_default_texture");
  
  // default fog configuration
  uniforms["gua_enable_fog"] = false;
  uniforms["gua_fog_start"] = 10.f;
  uniforms["gua_fog_end"] = 1000.f;

  // default tone mapping exposure
  uniforms["gua_tone_mapping_exposure"] = 1.0f;
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::background_mode(BackgroundMode mode) {
  uniforms["gua_background_mode"] = static_cast<int>(mode);;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription::BackgroundMode ResolvePassDescription::background_mode() const {
  auto uniform(uniforms.find("gua_background_mode"));
  return static_cast<ResolvePassDescription::BackgroundMode>(boost::get<int>(uniform->second.data));
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::background_color(utils::Color3f const& color) {
  uniforms["gua_background_color"] = color.vec3f();
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
utils::Color3f ResolvePassDescription::background_color() const {
  auto uniform(uniforms.find("gua_background_color"));
  return utils::Color3f(boost::get<math::vec3f>(uniform->second.data));
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::background_texture(std::string const& texture) {
  uniforms["gua_background_texture"] = texture;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::string ResolvePassDescription::background_texture() const {
  auto uniform(uniforms.find("gua_background_texture"));
  return boost::get<std::string>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::environment_lighting_spheremap(std::string const& spheremap_texture) {
  uniforms["gua_environment_lighting_spheremap"] = spheremap_texture;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& ResolvePassDescription::environment_lighting_spheremap() const {
  auto uniform(uniforms.find("gua_environment_lighting_spheremap"));
  return boost::get<std::string>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::environment_lighting_cubemap(std::string const& cube_map_positive_x,
  std::string const& cube_map_negative_x,
  std::string const& cube_map_positive_y,
  std::string const& cube_map_negative_y,
  std::string const& cube_map_positive_z,
  std::string const& cube_map_negative_z) {

  gua::Logger::LOG_WARNING << "ResolvePassDescription::environment_lighting_cubemap() : Using deafult texture. Opening not implemented yet." << std::endl;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::environment_lighting(utils::Color3f const& color) {
  uniforms["gua_environment_lighting_color"] = scm::math::vec3f(color.r(), color.g(), color.b());
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::environment_lighting_mode(EnvironmentLightingMode mode) {
  uniforms["gua_environment_lighting_mode"] = static_cast<int>(mode);
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription::EnvironmentLightingMode ResolvePassDescription::environment_lighting_mode() const {
  auto uniform(uniforms.find("gua_environment_lighting_mode"));
  return static_cast<ResolvePassDescription::EnvironmentLightingMode>(boost::get<int>(uniform->second.data));
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::ssao_enable(bool enable) {
  uniforms["gua_ssao_enable"] = enable;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool ResolvePassDescription::ssao_enable() const {
  auto uniform(uniforms.find("gua_ssao_enable"));
  return boost::get<bool>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::ssao_radius(float radius) {
  uniforms["gua_ssao_radius"] = radius;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float ResolvePassDescription::ssao_radius() const {
  auto uniform(uniforms.find("gua_ssao_radius"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::ssao_intensity(float intensity) {
  uniforms["gua_ssao_intensity"] = intensity;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float ResolvePassDescription::ssao_intensity() const {
  auto uniform(uniforms.find("gua_ssao_intensity"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::ssao_falloff(float falloff) {
  uniforms["gua_ssao_falloff"] = falloff;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float ResolvePassDescription::ssao_falloff() const {
  auto uniform(uniforms.find("gua_ssao_falloff"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::ssao_noise_texture(std::string const& texture) {
  uniforms["gua_noise_tex"] = texture;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& ResolvePassDescription::ssao_noise_texture() const {
  auto uniform(uniforms.find("gua_noise_tex"));
  return boost::get<std::string>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::screen_space_shadows(bool enable)
{
  uniforms["gua_screen_space_shadows"] = enable;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool ResolvePassDescription::screen_space_shadows() const
{
  auto uniform(uniforms.find("gua_screen_space_shadows"));
  return boost::get<bool>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::enable_fog(bool enable_fog) {
  uniforms["gua_enable_fog"] = enable_fog;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool ResolvePassDescription::enable_fog() const {
  auto uniform(uniforms.find("gua_enable_fog"));
  return boost::get<bool>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::fog_start(float fog_start) {
  uniforms["gua_fog_start"] = fog_start;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float ResolvePassDescription::fog_start() const {
  auto uniform(uniforms.find("gua_fog_start"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::fog_end(float fog_end) {
  uniforms["gua_fog_end"] = fog_end;
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
float ResolvePassDescription::fog_end() const {
  auto uniform(uniforms.find("gua_fog_end"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::tone_mapping_exposure(float value)
{
  uniforms["gua_tone_mapping_exposure"] = value;
  return *this; 
}

////////////////////////////////////////////////////////////////////////////////
float ResolvePassDescription::tone_mapping_exposure() const { 
  auto uniform(uniforms.find("gua_tone_mapping_exposure"));
  return boost::get<float>(uniform->second.data);
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::tone_mapping_method(ToneMappingMethod value) {
  tone_mapping_method_ = value; 
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription::ToneMappingMethod ResolvePassDescription::tone_mapping_method() const { 
  return tone_mapping_method_; 
}

////////////////////////////////////////////////////////////////////////////////
ResolvePassDescription& ResolvePassDescription::debug_tiles(bool value) {
  debug_tiles_ = value; 
  return *this;
}

////////////////////////////////////////////////////////////////////////////////
bool ResolvePassDescription::debug_tiles() const {
  return debug_tiles_; 
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PipelinePassDescription> ResolvePassDescription::make_copy() const {
  return std::make_shared<ResolvePassDescription>(*this);
}

////////////////////////////////////////////////////////////////////////////////
PipelinePass ResolvePassDescription::make_pass(RenderContext const& ctx, SubstitutionMap& substitution_map)
{
  substitution_map["gua_debug_tiles"] = debug_tiles() ? "1" : "0";
  substitution_map["gua_tone_mapping_method"] = std::to_string(static_cast<int>(tone_mapping_method()));

  PipelinePass pass{*this, ctx, substitution_map};
  return pass;
}

}
