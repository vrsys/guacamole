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
#include <gua/renderer/GBufferVideo3DUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/logger.hpp>
#include <gua/memory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ShaderProgram.hpp>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void GBufferVideo3DUberShader::create(std::set<std::string> const& material_names) {

  UberShaderFactory vshader_factory(
    ShadingModel::GBUFFER_VERTEX_STAGE, material_names
  );

  UberShaderFactory fshader_factory(
    ShadingModel::GBUFFER_FRAGMENT_STAGE, material_names,
    vshader_factory.get_uniform_mapping()
  );

  LayerMapping vshader_output_mapping = vshader_factory.get_output_mapping();

  fshader_factory.add_inputs_to_main_functions(
    {&vshader_output_mapping}, ShadingModel::GBUFFER_VERTEX_STAGE
  );

  UberShader::set_uniform_mapping(fshader_factory.get_uniform_mapping());
  UberShader::set_output_mapping(fshader_factory.get_output_mapping());

  // create depth shader
  std::vector<ShaderProgramStage> warp_pass_stages;
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _warp_pass_vertex_shader()));
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _warp_pass_geometry_shader()));
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _warp_pass_fragment_shader()));

  warp_pass_shader_ = gua::make_unique<ShaderProgram>();
  warp_pass_shader_->set_shaders(warp_pass_stages);
  warp_pass_shader_->save_to_file(".", "depth_pass");
  
  // create final shader
  std::vector<ShaderProgramStage> blend_pass_stages;
  blend_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _blend_pass_vertex_shader(vshader_factory, vshader_output_mapping)));
  blend_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _blend_pass_geometry_shader(vshader_factory, vshader_output_mapping)));
  blend_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _blend_pass_fragment_shader(fshader_factory, vshader_output_mapping)));

  // generate shader source
  set_shaders ( blend_pass_stages );
  save_to_file(".", "final_pass");
}

std::string const GBufferVideo3DUberShader::_warp_pass_vertex_shader() const
{
  return R"(
      // header
    #version 420
    #extension GL_NV_bindless_texture : require
    #extension GL_NV_gpu_shader5      : enable
    #extension GL_ARB_explicit_uniform_location : enable
    #extension GL_ARB_shading_language_420pack : enable
    #extension GL_EXT_texture_array : enable

    // input -----------------------------------------------------------------------
    layout(location=0) in vec3 gua_in_position;
    layout(location=1) in vec2 gua_in_texcoords;
    layout(location=2) in vec3 gua_in_normal;
    layout(location=3) in vec3 gua_in_tangent;
    layout(location=4) in vec3 gua_in_bitangent;

    // uniforms //=========> @ include something bla
    uniform mat4  gua_projection_matrix;
    uniform mat4  gua_view_matrix;
    uniform mat4  gua_model_matrix;
    uniform mat4  gua_normal_matrix;
    uniform mat4  gua_inverse_projection_view_matrix;
    uniform vec3  gua_camera_position;
    uniform float gua_texel_width;
    uniform float gua_texel_height;

    //calibration matrices
    uniform mat4  image_d_to_eye_d;
    uniform mat4  eye_d_to_world;
    uniform mat4  eye_d_to_eye_rgb;
    uniform mat4  eye_rgb_to_image_rgb;

    //kinect depths
    uniform sampler2DArray depth_video3d_texture;
    uniform sampler2DArray color_video3d_texture;

    uniform int layer;

    // outputs ---------------------------------------------------------------------
    out VertexData {
        vec3 normal;
        vec2 texture_coord;
        vec3 view_dir;
        vec3 pos_object_space;
        vec3 pos_eye_space;
        vec3 pos_d;
        float depth;
    } VertexOut;

    // main ------------------------------------------------------------------------
    void main() 
    {
      VertexOut.depth = texture2DArray(depth_video3d_texture, vec3(gua_in_texcoords.xy, layer)).r;

      vec4 POS_d = VertexOut.depth * image_d_to_eye_d * vec4(gua_in_position.xy, VertexOut.depth, 1.0);
      POS_d.z = VertexOut.depth;

      POS_d.w = 1.0;
      VertexOut.pos_d   = POS_d.xyz;

      vec4 POS_rgb = eye_d_to_eye_rgb * POS_d;

      if(POS_rgb.z > 0.0)
          VertexOut.texture_coord = (eye_rgb_to_image_rgb * vec4( (POS_rgb.xy/POS_rgb.z) ,0.0,1.0)).xy;
      else
          VertexOut.texture_coord = vec2(0.0);

      vec4 POS_ws =  eye_d_to_world * POS_d;

      ////////////////////////////////////////////////////////////////////////////////////
      VertexOut.normal           =  normalize(gua_inverse_projection_view_matrix * vec4(gua_in_normal, 0.0)).xyz;
      VertexOut.view_dir         = -normalize(gua_view_matrix * gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
      VertexOut.pos_object_space = POS_ws.xyz;
      VertexOut.pos_eye_space    = (gua_view_matrix * gua_model_matrix * POS_ws).xyz;

      gl_Position = gua_projection_matrix * gua_view_matrix * gua_model_matrix * POS_ws;
    }
    )";
}

std::string const GBufferVideo3DUberShader::_warp_pass_geometry_shader() const
{
#if 0
  std::string geometry_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_geom) //TODO gbuffer_video3d_video3d_vert
  );
#else
std::string geometry_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;

// material
uniform uint gua_material_id;

uint gua_get_material_id() {
  return gua_material_id;
}

// global variables
vec2 gua_texcoords;

vec3 gua_world_normal;
vec3 gua_world_position;
vec3 gua_world_tangent;
vec3 gua_world_bitangent;

vec3 gua_object_normal;
vec3 gua_object_position;
vec2 gua_object_texcoords;
vec3 gua_object_tangent;
vec3 gua_object_bitangent;

out vec3 gua_position_varying;
out vec2 gua_texcoords_varying;

float min_length = 0.0125; //TODO uniform
float geo_length = 0.01; //TODO uniform

// uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;
uniform float gua_texel_width;
uniform float gua_texel_height;

uniform int layer;

in VertexData {
    vec3 normal; // delete
    vec2 texture_coord;
    vec3 view_dir; // delete
    vec3 pos_object_space;
    vec3 pos_eye_space;
    vec3 pos_d; // delete
    float depth;
} VertexIn[3];

////// ----->  should come from @common/sampler_cast...
isampler2D gua_get_int_sampler(uvec2 handle) {
    return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler2D gua_get_uint_sampler(uvec2 handle) {
    return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_float_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_double_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
    return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

isampler3D gua_get_int_sampler3D(uvec2 handle) {
    return isampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler3D gua_get_uint_sampler3D(uvec2 handle) {
    return usampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_float_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_double_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}
//////// <-----

bool validSurface(vec3 a, vec3 b, vec3 c)
{
  float avg_depth = (VertexIn[0].depth + VertexIn[1].depth + VertexIn[2].depth)/3.0;
  float l = min_length * avg_depth + geo_length;
  if((length(a) > l) || (length(b) > l) || (length(c) > l)){
    return false;
  }
  return true;
}

vec3 computeNormal(vec3 x1, vec3 x2, vec3 x3)
{
    vec3 norm = cross(x3 - x1, x2 - x1);
    return normalize(norm);
}

void main()
{
  vec3 a = VertexIn[1].pos_eye_space - VertexIn[0].pos_eye_space;
  vec3 b = VertexIn[2].pos_eye_space - VertexIn[0].pos_eye_space;
  vec3 c = VertexIn[2].pos_eye_space - VertexIn[1].pos_eye_space;

  bool valid = validSurface(a,b,c);
  if (valid)
  {
      //vec3 normal = computeNormal(gl_in[0].gl_Position.xyz, gl_in[1].gl_Position.xyz, gl_in[2].gl_Position.xyz);
      vec3 normal = computeNormal(VertexIn[0].pos_object_space, VertexIn[1].pos_object_space, VertexIn[2].pos_object_space);
      for(int i = 0; i < gl_in.length(); i++)
      {
        gua_position_varying = vec3(0);

        gua_texcoords = VertexIn[i].texture_coord;
        gua_texcoords_varying = gua_texcoords;

        gua_object_normal =     normal;
        gua_object_tangent =    a;
        gua_object_bitangent =  b;
        gua_object_position =   VertexIn[i].pos_object_space;

        gua_world_normal =      normalize((gua_normal_matrix * vec4(normal, 0.0)).xyz);
        gua_world_tangent =     normalize((gua_normal_matrix * vec4(a, 0.0)).xyz);
        gua_world_bitangent =   normalize((gua_normal_matrix * vec4(b, 0.0)).xyz);
        gua_world_position =    (gua_model_matrix * vec4(VertexIn[i].pos_object_space, 1.0)).xyz;

        gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0);

        // done with the vertex
        EmitVertex();
      }
      EndPrimitive();
  }
}
  )";
#endif

  return geometry_shader;
}

std::string const GBufferVideo3DUberShader::_warp_pass_fragment_shader() const
{
#if 0

  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_frag)//TODO gbuffer_video3d_video3d_frag
    );

#else

std::string fragment_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

// input from vertex shader ----------------------------------------------------
in vec3 gua_position_varying;
in vec2 gua_texcoords_varying;

// uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;
uniform float gua_texel_width;
uniform float gua_texel_height;

//video3D
uniform int layer;
uniform sampler2DArray color_video3d_texture;

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

////// ----->  should come from @common/sampler_cast...
isampler2D gua_get_int_sampler(uvec2 handle) {
    return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler2D gua_get_uint_sampler(uvec2 handle) {
    return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_float_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_double_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
    return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

isampler3D gua_get_int_sampler3D(uvec2 handle) {
    return isampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler3D gua_get_uint_sampler3D(uvec2 handle) {
    return usampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_float_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_double_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}
//////// <-----

vec3 gua_get_position() {
  return gua_position_varying;
}

void gua_set_position(vec3 world_position) {
    vec4 pos = gua_projection_matrix * gua_view_matrix * vec4(world_position, 1.0);
    float ndc = pos.z/pos.w;
    gl_FragDepth = (((gl_DepthRange.diff) * ndc) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;
}

// main ------------------------------------------------------------------------
void main() {

  gl_FragDepth = gl_FragCoord.z;
}
)";

#endif

  return fragment_shader;
}

std::string const GBufferVideo3DUberShader::_blend_pass_vertex_shader(UberShaderFactory const& vshader_factory, LayerMapping const& vshader_output_mapping) const
{
#if 0 
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_vert) //TODO gbuffer_video3d_video3d_vert
  );
#else 
std::string vertex_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

// input -----------------------------------------------------------------------
layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

// uniforms //=========> @ include something bla
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;
uniform float gua_texel_width;
uniform float gua_texel_height;

//calibration matrices
uniform mat4  image_d_to_eye_d;
uniform mat4  eye_d_to_world;
uniform mat4  eye_d_to_eye_rgb;
uniform mat4  eye_rgb_to_image_rgb;

//kinect depths
uniform sampler2DArray depth_video3d_texture;
uniform sampler2DArray color_video3d_texture;

uniform int layer;

// outputs ---------------------------------------------------------------------
out VertexData {
    vec3 normal;
    vec2 texture_coord;
    vec3 view_dir;
    vec3 pos_object_space;
    vec3 pos_eye_space;
    vec3 pos_d;
    float depth;
} VertexOut;

// main ------------------------------------------------------------------------
void main() 
{
  VertexOut.depth = texture2DArray(depth_video3d_texture, vec3(gua_in_texcoords.xy, layer)).r;

  vec4 POS_d = VertexOut.depth * image_d_to_eye_d * vec4(gua_in_position.xy, VertexOut.depth, 1.0);
  POS_d.z = VertexOut.depth;

  POS_d.w = 1.0;
  VertexOut.pos_d   = POS_d.xyz;

  vec4 POS_rgb = eye_d_to_eye_rgb * POS_d;

  if(POS_rgb.z > 0.0)
      VertexOut.texture_coord = (eye_rgb_to_image_rgb * vec4( (POS_rgb.xy/POS_rgb.z) ,0.0,1.0)).xy;
  else
      VertexOut.texture_coord = vec2(0.0);

  vec4 POS_ws =  eye_d_to_world * POS_d;

  ////////////////////////////////////////////////////////////////////////////////////
  VertexOut.normal           =  normalize(gua_inverse_projection_view_matrix * vec4(gua_in_normal, 0.0)).xyz;
  VertexOut.view_dir         = -normalize(gua_view_matrix * gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  VertexOut.pos_object_space = POS_ws.xyz;
  VertexOut.pos_eye_space    = (gua_view_matrix * gua_model_matrix * POS_ws).xyz;

  gl_Position = gua_projection_matrix * gua_view_matrix * gua_model_matrix * POS_ws;
}
)";
#endif

  return vertex_shader;
}

std::string const GBufferVideo3DUberShader::_blend_pass_geometry_shader(UberShaderFactory const& vshader_factory, LayerMapping const& vshader_output_mapping) const
{
#if 0
  std::string geometry_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_geom) //TODO gbuffer_video3d_video3d_vert
  );
#else
std::string geometry_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;

// material
uniform uint gua_material_id;

uint gua_get_material_id() {
  return gua_material_id;
}

// material specific uniforms
@uniform_definition

// per-vertex output
@output_definition

// global variables
vec2 gua_texcoords;

vec3 gua_world_normal;
vec3 gua_world_position;
vec3 gua_world_tangent;
vec3 gua_world_bitangent;

vec3 gua_object_normal;
vec3 gua_object_position;
vec2 gua_object_texcoords;
vec3 gua_object_tangent;
vec3 gua_object_bitangent;

out vec3 gua_position_varying;
out vec2 gua_texcoords_varying;

float min_length = 0.0125; //TODO uniform
float geo_length = 0.01; //TODO uniform

// uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;
uniform float gua_texel_width;
uniform float gua_texel_height;

uniform int layer;

@material_methods

in VertexData {
    vec3 normal; // delete
    vec2 texture_coord;
    vec3 view_dir; // delete
    vec3 pos_object_space;
    vec3 pos_eye_space;
    vec3 pos_d; // delete
    float depth;
} VertexIn[3];

////// ----->  should come from @common/sampler_cast...
isampler2D gua_get_int_sampler(uvec2 handle) {
    return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler2D gua_get_uint_sampler(uvec2 handle) {
    return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_float_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_double_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
    return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

isampler3D gua_get_int_sampler3D(uvec2 handle) {
    return isampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler3D gua_get_uint_sampler3D(uvec2 handle) {
    return usampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_float_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_double_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}
//////// <-----

bool validSurface(vec3 a, vec3 b, vec3 c)
{
  float avg_depth = (VertexIn[0].depth + VertexIn[1].depth + VertexIn[2].depth)/3.0;
  float l = min_length * avg_depth + geo_length;
  if((length(a) > l) || (length(b) > l) || (length(c) > l)){
    return false;
  }
  return true;
}

vec3 computeNormal(vec3 x1, vec3 x2, vec3 x3)
{
    vec3 norm = cross(x3 - x1, x2 - x1);
    return normalize(norm);
}

void main()
{
  vec3 a = VertexIn[1].pos_eye_space - VertexIn[0].pos_eye_space;
  vec3 b = VertexIn[2].pos_eye_space - VertexIn[0].pos_eye_space;
  vec3 c = VertexIn[2].pos_eye_space - VertexIn[1].pos_eye_space;

  bool valid = validSurface(a,b,c);
  if (valid)
  {
      //vec3 normal = computeNormal(gl_in[0].gl_Position.xyz, gl_in[1].gl_Position.xyz, gl_in[2].gl_Position.xyz);
      vec3 normal = computeNormal(VertexIn[0].pos_object_space, VertexIn[1].pos_object_space, VertexIn[2].pos_object_space);
      for(int i = 0; i < gl_in.length(); i++)
      {
        // copy attributes
        // gl_Position = gl_in[i].gl_Position;

        gua_position_varying = vec3(0);

        gua_texcoords = VertexIn[i].texture_coord;
        gua_texcoords_varying = gua_texcoords;

        gua_object_normal =     normal;
        gua_object_tangent =    a;
        gua_object_bitangent =  b;
        gua_object_position =   VertexIn[i].pos_object_space;

        gua_world_normal =      normalize((gua_normal_matrix * vec4(normal, 0.0)).xyz);
        gua_world_tangent =     normalize((gua_normal_matrix * vec4(a, 0.0)).xyz);
        gua_world_bitangent =   normalize((gua_normal_matrix * vec4(b, 0.0)).xyz);
        gua_world_position =    (gua_model_matrix * vec4(VertexIn[i].pos_object_space, 1.0)).xyz;

        // big switch, one case for each material
        @material_switch

        gua_uint_gbuffer_varying_0.x = gua_material_id;
        gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0);

        // done with the vertex
        EmitVertex();
      }
      EndPrimitive();
  }
}
  )";
#endif

  // material specific uniforms
  string_utils::replace(geometry_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // output
  string_utils::replace(geometry_shader, "@output_definition",
    vshader_output_mapping.get_gbuffer_output_definition(false, true));

  // print material specific methods
  string_utils::replace(geometry_shader, "@material_methods",
    UberShader::print_material_methods(vshader_factory));

  // print main switch(es)
  string_utils::replace(geometry_shader, "@material_switch",
    UberShader::print_material_switch(vshader_factory));


  return geometry_shader;
  //TODO
}

std::string const GBufferVideo3DUberShader::_blend_pass_fragment_shader(UberShaderFactory const& fshader_factory, LayerMapping const& vshader_output_mapping) const
{

#if 0

  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_frag)//TODO gbuffer_video3d_video3d_frag
    );

#else

std::string fragment_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

// input from vertex shader ----------------------------------------------------
in vec3 gua_position_varying;
in vec2 gua_texcoords_varying;

@input_definition

// uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;
uniform float gua_texel_width;
uniform float gua_texel_height;

//video3D
uniform int layer;
uniform sampler2DArray color_video3d_texture;

// material specific uniforms
@uniform_definition

// outputs ---------------------------------------------------------------------
@output_definition

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

////// ----->  should come from @common/sampler_cast...
isampler2D gua_get_int_sampler(uvec2 handle) {
    return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler2D gua_get_uint_sampler(uvec2 handle) {
    return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_float_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_double_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
    return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

isampler3D gua_get_int_sampler3D(uvec2 handle) {
    return isampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler3D gua_get_uint_sampler3D(uvec2 handle) {
    return usampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_float_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_double_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}
//////// <-----

uint gua_get_material_id() {
  return gua_uint_gbuffer_varying_0.x;
}

vec3 gua_get_position() {
  return gua_position_varying;
}

void gua_set_position(vec3 world_position) {
    vec4 pos = gua_projection_matrix * gua_view_matrix * vec4(world_position, 1.0);
    float ndc = pos.z/pos.w;
    gl_FragDepth = (((gl_DepthRange.diff) * ndc) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;
}

// material specific methods
@material_methods

// main ------------------------------------------------------------------------
void main() {

  gl_FragDepth = gl_FragCoord.z;

  // big switch, one case for each material
  @material_switch

  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
)";

fragment_shader += fshader_factory.get_output_mapping().get_output_string("Video3D", "video_color");
fragment_shader +=  R"( = texture2DArray(color_video3d_texture, vec3(gua_texcoords_varying.xy, layer)).rgb;
}
)";

#endif

  // input from vertex shader
  string_utils::replace(fragment_shader, "@input_definition",
    vshader_output_mapping.get_gbuffer_output_definition(true, true));

  // material specific uniforms
  string_utils::replace(fragment_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(fragment_shader, "@output_definition",
    get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  // print material specific methods
  string_utils::replace(fragment_shader, "@material_methods",
    UberShader::print_material_methods(fshader_factory));

  // print main switch(es)
  string_utils::replace(fragment_shader, "@material_switch",
    UberShader::print_material_switch(fshader_factory));

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////
bool GBufferVideo3DUberShader::upload_to (RenderContext const& context) const
{
  // upload base class programs & upload depth program
  bool warp_program_uploaded  = warp_pass_shader_->upload_to(context);
  bool blend_program_uploaded = UberShader::upload_to(context);
/*
  if ( context.id > warp_color_result_.size() ) {
    warp_color_result_.resize(MAX_NUMBER_KINECT_);    
  }

  if ( context.id > warp_depth_result_.size() ) {
    warp_depth_result_.resize(MAX_NUMBER_KINECT_);
  }

  if ( context.id > warp_result_fbos_.size() ) {
    warp_result_fbos_.resize(MAX_NUMBER_KINECT_);
  }

  // initialize Texture Arrays (kinect depths & colors)
  warp_color_result_[context.id] = context.render_device->create_texture_2d(
                                                       scm::math::vec2ui(context.width, context.height),
                                                       scm::gl::FORMAT_RGBA_8,
                                                       0,
                                                       MAX_NUMBER_KINECT_,
                                                       1
                                                    );

  warp_depth_result_[context.id] = context.render_device->create_texture_2d( 
                                                       scm::math::vec2ui(context.width, context.height),
                                                       scm::gl::FORMAT_D32F,
                                                       0,
                                                       MAX_NUMBER_KINECT_,
                                                       1
                                                   );

  for ( auto& fbo : warp_result_fbos_[context.id] )
  {
    fbo = gua::make_unique<FrameBufferObject>();
    fbo->attach_color_buffers(0, warp_color_result_[context.id]);
  }

  warp_result_buffer_->create(context);
*/
  return warp_program_uploaded && blend_program_uploaded;
}

}

