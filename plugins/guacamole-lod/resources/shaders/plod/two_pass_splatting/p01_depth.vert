@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributms
layout(location = 0) in vec3 in_position;
layout(location = 5) in float in_radius;
layout(location = 6) in vec3 in_normal;

layout(location = 7) in int fem_vert_id_0;
layout(location = 8) in int fem_vert_id_1;
layout(location = 9) in int fem_vert_id_2;
layout(location = 10) in float fem_vert_w_0;
layout(location = 11) in float fem_vert_w_1;
layout(location = 12) in float fem_vert_w_2;

layout (std430, binding = 20) coherent readonly buffer time_series_data_ssbo {
  float[] time_series_data;
};


uniform uint gua_material_id;
uniform float radius_scaling;
uniform float max_surfel_radius;

uniform int floats_per_attribute_timestep;

uniform int attribute_offset;

uniform int current_timestep;

uniform float min_ssbo_value;
uniform float max_ssbo_value;

uniform float deform_factor = 500.0;

uniform bool use_programmable_attributes = true;
uniform bool use_programmable_deformation = true;

@include "../common/deformation.glsl"

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexOut;

void main() {

  @include "../common_LOD/PLOD_vertex_pass_through.glsl"


  int timestep_offset = current_timestep * floats_per_attribute_timestep;

  vec3 read_position = in_position;

  if(use_programmable_attributes) {

  	if(use_programmable_deformation) {
      deform_position(read_position);
  	}
  }

  gl_Position = vec4(read_position, 1.0);



  VertexOut.pass_normal = in_normal;
}
