@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec2 quad_coords;

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_global_variable_declaration.glsl"
///////////////////////////////////////////////////////////////////////////////
layout(binding=0) uniform sampler2D p02_color_texture;
layout(binding=3) uniform sampler2D p02_weight_and_depth_texture;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_fragment_shader_output.glsl"

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  ivec2 current_fragment_pos = ivec2(gl_FragCoord.xy);

  vec2 accumulated_weight_and_depth = texelFetch(p02_weight_and_depth_texture, current_fragment_pos, 0).rg;
  float accumulated_weight = accumulated_weight_and_depth.x;


    float accumulated_depth =  accumulated_weight_and_depth.y ;
    float blended_depth = accumulated_depth / accumulated_weight;

    vec3  normalized_color  = vec3(1.0);
    vec3 coords = vec3(quad_coords, 0.0);

    vec3 accumulated_color = texelFetch(p02_color_texture, current_fragment_pos, 0).rgb;
   

    normalized_color = accumulated_color.rgb / accumulated_weight ;

    gua_color = normalized_color.rgb;
    gua_normal = vec3(0.0, 0.0, 1.0);//normalized_normal;

    gua_metalness  = 0.0;
    gua_roughness  = 1.0;
    gua_emissivity = 1.0;
    gua_alpha      = accumulated_weight;
    gua_flags_passthrough = true;//(gua_emissivity > 0.99999);

    // calculate world position from blended depth
    //vec4 world_pos_h = gua_inverse_projection_view_matrix * vec4(gl_FragCoord.xy, depth_visibility_pass, 1.0);

    if( (accumulated_weight == 0.0) || !(blended_depth >= 0.0 && blended_depth  <= 0.9999999) )
      discard;
      //blended_depth = 0.0;

    //blended_depth = 0.2;


    vec4 world_pos_h = gua_inverse_projection_view_matrix * vec4(gl_FragCoord.xy, blended_depth, 1.0);
    gua_world_position = world_pos_h.xyz/world_pos_h.w;

    //gua_world_position = vec3(1.0, 0.0, 0.0);

    gl_FragDepth = blended_depth; 



    @include "common/gua_write_gbuffer.glsl"

  //gua_out_color = vec3(1.0, 0.0, 0.0);
}

