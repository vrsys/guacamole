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
in vec3 pass_point_color;
in vec3 pass_normal;
in vec2 pass_uv_coords;

@include "common/gua_fragment_shader_input.glsl"

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////

layout (location = 0) out vec4 out_accumulated_color;
layout (location = 1) out vec4 out_accumulated_normal;
layout (location = 2) out vec3 out_accumulated_pbr;

///////////////////////////////////////////////////////////////////////////////
// splatting methods
///////////////////////////////////////////////////////////////////////////////



@material_uniforms@

@include "common/gua_global_variable_declaration.glsl"

float weight = 0;

@material_method_declarations_frag@


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {
  vec2 uv_coords = pass_uv_coords;

  float normalAdjustmentFactor = 1.0;

  //turn normal to viewer
  if (pass_normal.z < 0.0) {
    normalAdjustmentFactor = -1.0;
  }

  if( dot(uv_coords, uv_coords) > 1) 
    discard;
  else
    weight = gaussian[(int)(round(length(uv_coords) * 31.0))];


  @include "common/gua_global_variable_assignment.glsl"

  gua_color = pass_point_color;
  gua_normal = pass_normal;
  gua_metalness  = 0.0;
  gua_roughness  = 1.0;
  gua_emissivity = 1.0; // pass through if unshaded


  @material_input@
  @material_method_calls_frag@

  out_accumulated_color = vec4(weight * gua_color, weight);
  out_accumulated_normal = normalAdjustmentFactor * vec4(weight * gua_normal, gl_FragCoord.z);

  out_accumulated_pbr = vec3(gua_metalness, gua_roughness, gua_emissivity) * weight;


}

