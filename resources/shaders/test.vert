@include "shaders/common/header.glsl"

layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

uniform mat4 gua_transform;
uniform mat4 gua_normal_transform;
uniform mat4 gua_projection_matrix;
uniform mat4 gua_view_matrix;

out vec3 gua_normal;

void main() {
  gua_normal = (gua_normal_transform * vec4(gua_in_normal, 0.0)).xyz;
  gl_Position = gua_projection_matrix * gua_view_matrix * gua_transform * vec4(gua_in_position, 1.0);
}
