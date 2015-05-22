@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_abuffer.glsl"

flat in uint data_pos;

// output
layout(location=0) out vec3 gua_out_color;

void main() {
  
  uvec4 data = frag_data[data_pos];
    
  vec3 color = vec3(unpackUnorm2x16(data.x), unpackUnorm2x16(data.y).x);
  vec3 normal = vec3(unpackSnorm2x16(data.y).y, unpackSnorm2x16(data.z));

  gua_out_color = normal.rgb;
}