@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@include "../common/vertex_layout.glsl"

layout (std430, binding = 20) coherent readonly buffer time_series_data_ssbo {
  float[] time_series_data;
};

uniform int attribute_offset;

uniform uint gua_material_id;
uniform float radius_scaling;
uniform float max_surfel_radius;

uniform int floats_per_attribute_timestep;
uniform float current_timestep;

uniform float min_ssbo_value;
uniform float max_ssbo_value;

uniform int attribute_to_visualize = 0;
uniform float deform_factor = 500.0;

uniform float mix_in_factor = 0.7;

uniform bool use_programmable_attributes = true;

uniform bool enable_time_series_deformation = true;
uniform bool enable_time_series_coloring = true;

uniform bool enable_linear_temporal_interpolation = true;

@include "../common/coloring.glsl"
@include "../common/deformation.glsl"







out VertexData {
  //output to geometry shader
  vec3 pass_ms_u;
  vec3 pass_ms_v;

  vec3 pass_point_color;
  vec3 pass_normal;
} VertexOut;




void main() {
  @include "../common_LOD/PLOD_vertex_pass_through.glsl"

  vec3 read_position = in_position;

  vec3 raw_point_color = vec3(in_r, in_g, in_b); 

  if( use_programmable_attributes && 
      (! (    (fem_vert_w_0 <= 0.0)  
           || (fem_vert_w_1 <= 0.0)  
           || (fem_vert_w_2 <= 0.0) ) )
    ) {
    
    int timestep_offset = int(current_timestep) * floats_per_attribute_timestep;

    if(enable_time_series_deformation) {
      deform_position(read_position);
    }

    if( enable_time_series_coloring ) {
      float mixed_value =   fem_vert_w_0 * time_series_data[attribute_offset * attribute_to_visualize + timestep_offset + fem_vert_id_0]
                          + fem_vert_w_1 * time_series_data[attribute_offset * attribute_to_visualize + timestep_offset + fem_vert_id_1]
                          + fem_vert_w_2 * time_series_data[attribute_offset * attribute_to_visualize + timestep_offset + fem_vert_id_2];

      VertexOut.pass_point_color = mix(data_value_to_rainbow(mixed_value, min_ssbo_value, max_ssbo_value), raw_point_color, mix_in_factor);
    } else {
      VertexOut.pass_point_color = raw_point_color;
    }

  } else {
    VertexOut.pass_point_color = raw_point_color;
  }


  gl_Position = vec4(read_position, 1.0);

  VertexOut.pass_normal = in_normal;


}