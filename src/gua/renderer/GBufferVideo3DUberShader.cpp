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

  // VERTEX SHADER -------------------------------------------------------------
  //see _final_vertex_shader(UberShaderFactory const& vshader_factory)

  // FRAGMENT SHADER -----------------------------------------------------------
  //see _final_fragment_shader(UberShaderFactory const& fshader_factory)

  std::vector<ShaderProgramStage> shader_stages;
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _final_vertex_shader(vshader_factory, vshader_output_mapping)));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _final_geometry_shader(vshader_factory, vshader_output_mapping)));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _final_fragment_shader(fshader_factory, vshader_output_mapping)));

  // generate shader source
  set_shaders ( shader_stages );

  //save shader sources for debugging
  save_to_file(".", "final_pass");
}

std::string const GBufferVideo3DUberShader::_final_vertex_shader(UberShaderFactory const& vshader_factory, LayerMapping const& vshader_output_mapping) const
{
#if 0
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_mesh_vert) //TODO gbuffer_video3d_video3d_vert
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
//uniform sampler2DArray depth_video3d_texture;
//uniform sampler2DArray color_video3d_texture;
uniform sampler2D depth_video3d_texture;
uniform sampler2D color_video3d_texture;

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
void main() {
  //VertexOut.depth = texture2DArray(depth_video3d_texture, vec3(gua_in_texcoords.xy, 0)).r;
  VertexOut.depth = texture(depth_video3d_texture, gua_in_texcoords.xy).r;

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

  if(POS_ws.y < 0.0)
    POS_ws.y = 0.0;

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

std::string const GBufferVideo3DUberShader::_final_geometry_shader(UberShaderFactory const& vshader_factory, LayerMapping const& vshader_output_mapping) const
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
      vec3 normal = computeNormal(gl_in[0].gl_Position.xyz, gl_in[1].gl_Position.xyz, gl_in[2].gl_Position.xyz);
      for(int i = 0; i < gl_in.length(); i++)
      {
        // copy attributes
        // gl_Position = gl_in[i].gl_Position;

        gua_position_varying = vec3(0);

        gua_texcoords = VertexIn[i].texture_coord;

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
#endif

  return geometry_shader;
  //TODO
}

std::string const GBufferVideo3DUberShader::_final_fragment_shader(UberShaderFactory const& fshader_factory, LayerMapping const& vshader_output_mapping) const
{
  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_mesh_frag)//TODO gbuffer_video3d_video3d_frag
    );

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

}

