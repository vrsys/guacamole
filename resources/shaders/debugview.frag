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

  int shadow_debug_size  = 150;

  if ( fragment_position.y / debug_window_height == 0)
  {
    vec2 texcoord  = vec2(float(mod(fragment_position.x, debug_window_width)) / debug_window_width, 
                          float(mod(fragment_position.y, debug_window_height)) / debug_window_height);
                           
    // output depth
    if ( fragment_position.x / debug_window_width == 0 ) {
      gua_out_color = vec3(gua_get_depth(texcoord));
    }

    // output color
    if ( fragment_position.x / debug_window_width == 1 ) {
      gua_out_color = gua_get_color(texcoord);
    }

    // output normal
    if ( fragment_position.x / debug_window_width == 2 ) {
      gua_out_color = gua_get_normal(texcoord);
    }

    // output position
    if ( fragment_position.x / debug_window_width == 3 ) {
      gua_out_color = gua_get_position(texcoord);
    }

    if ( fragment_position.x / debug_window_width == 4 ) {

      uint nlights = gua_sun_lights_num;
      int bitset_words = ((gua_lights_num - 1) >> 5) + 1;

      ivec2 tile = ivec2(mod(fragment_position.x, debug_window_width ), 
                         mod(fragment_position.y, debug_window_height ));

      tile = 5 * tile >> @light_table_tile_power@;
                  
      for (int sl = 0; sl < bitset_words; ++sl) {
        nlights += bitCount(texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r);
      }
      gua_out_color = vec3(float(nlights) / gua_lights_num);
    }

  } else if (fragment_position.x / shadow_debug_size == 0 && fragment_position.y >= debug_window_height) {

    int counter = 0;
    int shadow_map = (fragment_position.y - debug_window_height) / shadow_debug_size;

    for (int i = 0; i < gua_lights_num; ++i) {
      if (gua_lights[i].casts_shadow) {
        if (shadow_map > counter) {
          ++counter;
        } else if (shadow_map == counter) {
          vec2 texcoord = vec2(float(mod(fragment_position.x, shadow_debug_size)) / shadow_debug_size, 
                               float(mod(fragment_position.y-debug_window_height, shadow_debug_size)) / shadow_debug_size);
          gua_out_color = vec3(texture(sampler2D(gua_lights[i].shadow_map), texcoord).r);
          return;
        }
      }
    }

    if (shadow_map+1 > counter) {
      discard;
    }

  } else {
    discard;
  }
  
}

