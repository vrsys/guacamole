@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

// input attributes
layout (location = 0) in vec3  in_position;
layout (location = 1) in float in_r;
layout (location = 2) in float in_g;
layout (location = 3) in float in_b;
layout (location = 4) in float empty;
layout (location = 5) in float in_radius;
layout (location = 6) in vec3 in_normal;

layout(location = 7) in int fem_vert_id_0;
layout(location = 8) in int fem_vert_id_1;
layout(location = 9) in int fem_vert_id_2;
layout(location = 10) in float fem_vert_w_0;
layout(location = 11) in float fem_vert_w_1;
layout(location = 12) in float fem_vert_w_2;

layout (std430, binding = 4) buffer time_series_data_ssbo {
  float[] time_series_data;
};


uniform uint gua_material_id;
uniform float radius_scaling;
uniform float max_surfel_radius;

uniform int floats_per_timestep;

uniform float min_ssbo_value;
uniform float max_ssbo_value;

out VertexData {
  //output to geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;

  vec3 pass_point_color;
  vec3 pass_normal;
} VertexOut;


void main() {
  @include "../common_LOD/PLOD_vertex_pass_through.glsl"

  VertexOut.pass_point_color = vec3(in_r, in_g, in_b);


  //if(fem_vert_w_0 != 0.0) {
  if( ! (    (fem_vert_w_0 <= 0.0)  
          || (fem_vert_w_1 <= 0.0)  
          || (fem_vert_w_2 <= 0.0)
      )
     ) {
    
    int timestep_offset = 30 * floats_per_timestep;

    int attribute_offset = floats_per_timestep / 6;

    int access_id = gl_VertexID;

    float mixed_value =   fem_vert_w_0 * time_series_data[/*timestep_offset + attribute_offset * 3 +*/ fem_vert_id_0]
                        + fem_vert_w_1 * time_series_data[/*timestep_offset + attribute_offset * 3 +*/ fem_vert_id_1]
                        + fem_vert_w_2 * time_series_data[/*timestep_offset + attribute_offset * 3 +*/ fem_vert_id_2];

    //VertexOut.pass_point_color = vec3(0.01 * (mixed_value), 0.0, 0.0);
    //VertexOut.pass_point_color = vec3(1.0, 0.0, 0.0);
    
    float normalized_value = (mixed_value) / (max_ssbo_value );
    VertexOut.pass_point_color = vec3(normalized_value, 0.0, 0.0);

    if(mixed_value > 0.0 && mixed_value < max_ssbo_value / 2.0) {
      VertexOut.pass_point_color = vec3(0.0, 1.0, 1.0);
    }

  } else {

    //VertexOut.pass_point_color = vec3(0.0, 1.0, 0.0);
  }


  VertexOut.pass_normal = in_normal;

}