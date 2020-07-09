@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

@include "../common/vertex_layout.glsl"


layout (std430, binding = 20) coherent readonly buffer time_series_data_ssbo {
  float[] time_series_data;
};

//layout (std430, binding = 20) coherent readonly buffer surfel_data_ssbo {
//  float[] time_series_data;
//};

uniform int floats_per_attribute_timestep;
uniform float current_timestep;

uniform int attribute_to_visualize = 0;
uniform float deform_factor = 500.0;

uniform float mix_in_factor = 0.7;

uniform bool use_programmable_attributes = true;

uniform bool enable_time_series_deformation = true;
uniform bool enable_time_series_coloring = true;


uniform bool enable_linear_temporal_interpolation = true;

uniform int attribute_offset;

uniform bool has_provenance_attributes;

@include "../common/coloring.glsl"
@include "../common/deformation.glsl"



out VertexData {
  vec3 pass_normal;
  vec3 pass_point_color;
  float pass_radius;
#if @get_enable_multi_view_rendering@
  int layer_id;
#endif
} VertexOut;



void main() {

  VertexOut.pass_point_color  = vec3(in_r, in_g, in_b);
  VertexOut.pass_normal = in_normal; 
  VertexOut.pass_radius = in_radius;

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

	    if( enable_time_series_coloring ) {
	      vec3 attribute_color = sample_attribute_color();

	      VertexOut.pass_point_color = mix(attribute_color, raw_point_color, mix_in_factor);
	    } else {
	      VertexOut.pass_point_color = raw_point_color;
	    }

	} else {
	  VertexOut.pass_point_color = raw_point_color;

    }

  gl_Position = vec4(read_position, 1.0);

  VertexOut.pass_normal = in_normal;





#if @get_enable_multi_view_rendering@
#if @get_enable_hardware_mvr@
VertexOut.layer_id = gl_ViewID_OVR;
#else
VertexOut.layer_id = gl_InstanceID;
#endif
#endif

}
