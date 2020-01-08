@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@include "../common/vertex_layout.glsl"

layout (std430, binding = 20) coherent readonly buffer time_series_data_ssbo {
  float[] time_series_data;
};


uniform uint gua_material_id;
uniform float radius_scaling;
uniform float max_surfel_radius;

uniform int floats_per_attribute_timestep;

uniform int attribute_offset;

uniform float current_timestep;

uniform float min_ssbo_value;
uniform float max_ssbo_value;

uniform float deform_factor = 500.0;

uniform bool use_programmable_attributes = true;
uniform bool enable_time_series_deformation = true;

uniform bool enable_linear_temporal_interpolation = true;

@include "../common/deformation.glsl"

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexOut;

void main() {

  @include "../common_LOD/PLOD_vertex_pass_through.glsl"


  int timestep_offset = int(current_timestep) * floats_per_attribute_timestep;

  vec3 read_position = in_position;

  if(use_programmable_attributes) {

  	if(enable_time_series_deformation) {
      deform_position(read_position);
  	}
  }

  gl_Position = vec4(read_position, 1.0);



  VertexOut.pass_normal = in_normal;
}
