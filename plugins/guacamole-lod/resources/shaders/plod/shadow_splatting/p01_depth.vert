@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@include "../common/vertex_layout.glsl"

layout (std430, binding = 20) coherent readonly buffer time_series_data_ssbo {
  float[] time_series_data;
};

uniform float deform_factor = 500.0;

@include "../common/deformation.glsl"

uniform bool use_programmable_attributes = true;

uniform bool enable_time_series_deformation = true;

uniform bool enable_linear_temporal_interpolation = true;

uniform float radius_scaling;
uniform float max_surfel_radius;

uniform bool has_provenance_attributes;

out VertexData {
  vec3 pass_ms_u;
  vec3 pass_ms_v;
  vec3 pass_normal;
} VertexOut;

void main() {

@include "../common_LOD/PLOD_vertex_pass_through.glsl"

vec3 read_position = in_position;

vec3 raw_point_color = vec3(in_r, in_g, in_b); 

if( has_provenance_attributes && use_programmable_attributes && 
  (! (    (fem_vert_w_0 <= 0.0)  
       || (fem_vert_w_1 <= 0.0)  
       || (fem_vert_w_2 <= 0.0) ) )
) {
    int timestep_offset = int(current_timestep) * floats_per_attribute_timestep;

    if(enable_time_series_deformation) {
      deform_position(read_position);
    }
}

gl_Position = vec4(read_position, 1.0);


}

