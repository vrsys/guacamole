@include "common/header.glsl"

layout(early_fragment_tests) in;



@include "common/gua_camera_uniforms.glsl"

const float gaussian[32] = float[](
  1.000000, 1.000000, 0.988235, 0.968627, 0.956862, 0.917647, 0.894117, 0.870588, 0.915686, 0.788235,
  0.749020, 0.690196, 0.654902, 0.619608, 0.552941, 0.513725, 0.490196, 0.458824, 0.392157, 0.356863,
  0.341176, 0.278431, 0.254902, 0.227451, 0.188235, 0.164706, 0.152941, 0.125490, 0.109804, 0.098039,
  0.074510, 0.062745
);

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in VertexData {
  in vec3 pass_point_color;
  in vec3 pass_normal;
  in vec2 pass_uv_coords;
  in float pass_log_depth;
} VertexIn;

@include "common/gua_fragment_shader_input.glsl"


uniform mat4 inverse_view_matrix;

///////////////////////////////////////////////////////////////////////////////
// sampler
///////////////////////////////////////////////////////////////////////////////
layout(binding=0) uniform sampler2D p01_linear_depth_texture;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

layout (location = 0) out vec3 out_accumulated_color;
layout (location = 1) out vec3 out_accumulated_normal;
layout (location = 2) out vec3 out_accumulated_pbr;
layout (location = 3) out vec2 out_accumulated_weight_and_depth;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////



@material_uniforms@

@include "common/gua_global_variable_declaration.glsl"

float weight = 0;

@material_method_declarations_frag@


//layout (std430, binding = 20) buffer dummy_value_ssbo_layout{
//  float[] dummy_values;
//};


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  vec2 uv_coords = VertexIn.pass_uv_coords;

  //turn normal to viewer
  vec4 view_normal = inverse_view_matrix * vec4(VertexIn.pass_normal, 0.0);
  vec3 face_forward_normal = VertexIn.pass_normal.xyz;

  if (view_normal.z < 0.0) {
    face_forward_normal = -face_forward_normal;
  }

  if( dot(uv_coords, uv_coords) > 1) 
    discard;
  else
    weight = gaussian[(int)(round(length(uv_coords) * 31.0))];


  for (int i=0; i < gua_clipping_plane_count; ++i) {
    if (dot(gua_clipping_planes[i].xyz, gua_varying_world_position.xyz) + gua_clipping_planes[i].w < 0) {
      discard;
    }
  }


  @include "common/gua_global_variable_assignment.glsl"

  gua_color      = VertexIn.pass_point_color;
  gua_normal     = VertexIn.pass_normal;
  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 1.0; // pass through if unshaded

  @material_input@
  @material_method_calls_frag@

  out_accumulated_color  = vec3(weight * gua_color);
  out_accumulated_normal = vec3(weight * face_forward_normal);
  out_accumulated_pbr    = vec3(gua_metalness, gua_roughness, gua_emissivity) * weight;



  //out_accumulated_color = vec3(dummy_values[int(gl_FragCoord.x) % 250] / 250, 0.0, 0.0);

  //out_accumulated_normal = vec3(0.0, 0.0, 1.0);

  out_accumulated_weight_and_depth = vec2(weight, weight * VertexIn.pass_log_depth);
}

