@include "common/header.glsl"

// input
layout(location=0) in vec3 gua_in_position;

// uniforms
@include "common/gua_camera_uniforms.glsl"

out flat int is_for_right_eye;

void main() {
  // gl_Position = gua_model_view_projection_matrix * vec4(gua_in_position, 1.0);

#if @get_enable_multi_view_rendering@
	if(0 == gl_InstanceID) {
#endif
		
		gl_Position = gua_model_view_projection_matrix * vec4(gua_in_position, 1.0);

#if @get_enable_multi_view_rendering@
	} else {
  		gl_Position = gua_secondary_model_view_projection_matrix * vec4(gua_in_position, 1.0);
  		
	}

    is_for_right_eye = gl_InstanceID;
#endif


}
