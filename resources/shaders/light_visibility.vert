@include "common/header.glsl"

#if @is_hardware_multi_view_rendering_enabled@
#extension GL_OVR_multiview2: require
layout(num_views = 2) in;
#endif //is_hardware_multi_view_rendering_enabled


// input
layout(location=0) in vec3 gua_in_position;

// uniforms
@include "common/gua_camera_uniforms.glsl"

#if @is_hardware_multi_view_rendering_enabled@ || @get_enable_multi_view_rendering@
out flat int is_for_right_eye;
#endif // endif: any form of MVR enabled

void main() {

#if @is_hardware_multi_view_rendering_enabled@
  int viewport_index = int(gl_ViewID_OVR);
#elif @get_enable_multi_view_rendering@
  int viewport_index = gl_InstanceID;
#endif //is_hardware_multi_view_rendering_enabled

#if @get_enable_multi_view_rendering@
  if(0 == viewport_index) {
#endif // endif: any form of MVR enabled

	gl_Position = gua_model_view_projection_matrix * vec4(gua_in_position, 1.0);

#if @get_enable_multi_view_rendering@
	} else {
  		gl_Position = gua_secondary_model_view_projection_matrix * vec4(gua_in_position, 1.0);
	}
    is_for_right_eye = viewport_index;
#endif
}
