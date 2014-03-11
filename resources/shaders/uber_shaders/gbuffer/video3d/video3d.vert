// header ----------------------------------------------------------------------
@include "shaders/common/header.glsl"
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_texture_array : enable

// input -----------------------------------------------------------------------
layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

// uniforms
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

//calibration matrices
uniform mat4  image_d_to_eye_d;
uniform mat4  eye_d_to_world;
uniform mat4  eye_d_to_eye_rgb;
uniform mat4  eye_rgb_to_image_rgb;

//kinect depths
uniform sampler2DArray depth_video3d_texture;
uniform sampler2DArray color_video3d_texture;
//uniform sampler2D depth_video3d_texture;
//uniform sampler2D color_video3d_texture;

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
	VertexOut.depth = texture2DArray(depth_video3d_texture, vec3(gua_in_texcoords.xy, 0)).r;
  //VertexOut.depth = texture(depth_video3d_texture, gua_in_texcoords.xy).r;

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

  VertexOut.normal           =  normalize(gua_inverse_projection_view_matrix * vec4(gua_in_normal, 0.0)).xyz;
  VertexOut.view_dir         = -normalize(gua_view_matrix * gua_model_matrix * vec4(gua_in_position, 1.0)).xyz;
  VertexOut.pos_object_space = POS_ws.xyz;
  VertexOut.pos_eye_space    = (gua_view_matrix * gua_model_matrix * POS_ws).xyz;

  gl_Position = gua_projection_matrix * gua_view_matrix * gua_model_matrix * POS_ws;
}
