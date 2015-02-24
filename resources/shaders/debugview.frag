@include "common/header.glsl"

// varyings
in vec2 gua_quad_coords;

@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"
@include "common/gua_shading.glsl"


// output
layout(location=0) out vec3 gua_out_color;

void main() {

  ivec2 fragment_position = ivec2(gl_FragCoord.xy);
  const int number_of_gbuffers = 5;

  int debug_window_width  = int(gua_resolution.x / number_of_gbuffers);
  int debug_window_height = int((debug_window_width * gua_resolution.y) / gua_resolution.x);

  if ( fragment_position.y / debug_window_height == 0)
  {
    vec2 texcoord  = vec2(float(mod(fragment_position.x, debug_window_width)) / debug_window_width, 
                          float(mod(fragment_position.y, debug_window_height)) / debug_window_height);
                           
    // output depth
    if ( fragment_position.x / debug_window_width == 0 ) {
      gua_out_color = vec3(texture2D(sampler2D(gua_gbuffer_depth), texcoord).x * 2.0 - 1.0);
    }

    // output color
    if ( fragment_position.x / debug_window_width == 1 ) {
      gua_out_color = texture2D(sampler2D(gua_gbuffer_color), texcoord).rgb;
    }

    // output normal
    if ( fragment_position.x / debug_window_width == 2 ) {
      gua_out_color = texture2D(sampler2D(gua_gbuffer_normal), texcoord).rgb * 2 - 1;
    }

    // output position
    if ( fragment_position.x / debug_window_width == 3 ) {
      vec4 screen_space_pos = vec4(texcoord * 2.0 - 1.0, texture2D(sampler2D(gua_gbuffer_depth), texcoord).x, 1.0);
      vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
      h /= h.w;
      gua_out_color = h.xyz;
    }

    if ( fragment_position.x / debug_window_width == 4 ) {

      uint nlights = 0;
      int bitset_words = ((gua_lights_num - 1) >> 5) + 1;

      ivec2 tile = ivec2(mod(fragment_position.x, debug_window_width ), 
                         mod(fragment_position.y, debug_window_height ));

      tile = 5 * tile >> @light_table_tile_power@;
                  
      for (int sl = 0; sl < bitset_words; ++sl) {
        nlights += bitCount(texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r);
      }
      gua_out_color = vec3(float(nlights) / gua_lights_num);
    }

  } else {
    discard;
  }
  
}

