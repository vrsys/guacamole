@include "common/header.glsl"

#if @get_enable_multi_view_rendering@
#extension GL_OVR_multiview2: require
layout(num_views = 2) in;
#endif


// input
layout(location=0) in vec3 gua_in_position;

// uniforms
@include "common/gua_camera_uniforms.glsl"

out flat int is_for_right_eye;

void main() {
  // gl_Position = gua_model_view_projection_matrix * vec4(gua_in_position, 1.0);


#if @get_enable_multi_view_rendering@
  int viewport_index = 0;
  if(1 == gua_camera_in_multi_view_rendering_mode) {
    viewport_index = gl_InstanceID;
  //test_color = vec3(0.0, 1.0, 0.0);
  } 
  if(1 == gua_hardware_multi_view_rendering_mode_enabled) {
    viewport_index = int(gl_ViewID_OVR);
  //test_color = vec3(0.0, 0.0, 1.0);
  }

  if(0 == viewport_index) {
#endif
		
	gl_Position = gua_model_view_projection_matrix * vec4(gua_in_position, 1.0);

#if @get_enable_multi_view_rendering@
	} else {
  		gl_Position = gua_secondary_model_view_projection_matrix * vec4(gua_in_position, 1.0);
  		
	}

    is_for_right_eye = viewport_index;
#endif


}
