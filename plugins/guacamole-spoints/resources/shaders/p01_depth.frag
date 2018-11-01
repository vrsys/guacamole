@include "common/header.glsl"

@include "common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////

in vec2 uv_coords;
//in float es_half_cube_side_length;
//in float eye_space_depth;
void main() {

  if( dot(uv_coords, uv_coords) >= 1.0)
    discard;



  //float epsilon = //(gua_clip_far - gua_clip_near) / 100000.0;
  //gl_FragDepth = (-(get_lin_eyespace_depth_from_fragment() + es_half_cube_side_length)) / gua_clip_far;//((gl_FragCoord.z * gua_clip_far) / gua_clip_far) + epsilon;

  //gl_FragDepth = gl_FragCoord.z;
  //how far should it blend? Voxels overlap slightly and furthermore should not occupy the same space; -> move at most 1 voxel length (for now hardcode to something)
}

