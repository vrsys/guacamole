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

  if ( fragment_position.y < debug_window_height)
  {
    vec2 original_texcoord  = vec2(float(mod(fragment_position.x, debug_window_width)) / debug_window_width,
                                   float(mod(fragment_position.y, debug_window_height)) / debug_window_height);

    vec2 lookup_texcoord = original_texcoord;
    #if @get_enable_multi_view_rendering@

      if(1 == gua_camera_in_multi_view_rendering_mode) {
        lookup_texcoord.x *= 0.5;
      }
    #endif

    // output depth
    if ( fragment_position.x < debug_window_width) {
      gua_out_color = vec3(gua_get_depth(lookup_texcoord));
    } else if ( fragment_position.x < 2*debug_window_width) {
        // output color
      gua_out_color = gua_get_color(lookup_texcoord);
    } else if ( fragment_position.x < 3*debug_window_width) {
        // output normal
      gua_out_color = gua_get_normal(lookup_texcoord);
    } else if ( fragment_position.x < 4*debug_window_width) {
        // output position
      gua_out_color = gua_get_position(lookup_texcoord);
    } else if ( fragment_position.x < 5*debug_window_width) {
      unsigned int nlights = gua_sun_lights_num;
      int bitset_words = ((gua_lights_num - 1) >> 5) + 1;

      ivec2 tile = ivec2(mod(fragment_position.x, debug_window_width ),
                         mod(fragment_position.y, debug_window_height ));

      tile = 5 * tile >> @light_table_tile_power@;

      for (int sl = 0; sl < bitset_words; ++sl) {
        nlights += bitCount(texelFetch(usampler3D(gua_light_bitset), ivec3(tile, sl), 0).r);
      }
      gua_out_color = vec3(float(nlights) / gua_lights_num);
    }

#if @get_enable_multi_view_rendering@
  else if(1 == gua_camera_in_multi_view_rendering_mode) {
    lookup_texcoord.x = (original_texcoord.x * 0.5) + 0.5;
    if ( fragment_position.x < 6 * debug_window_width) {
      gua_out_color = vec3(gua_get_depth(lookup_texcoord));
    } else if (fragment_position.x < 7 * debug_window_width) {
      gua_out_color = vec3(gua_get_color(lookup_texcoord));
    } else if (fragment_position.x < 8 * debug_window_width) {
      gua_out_color = vec3(gua_get_normal(lookup_texcoord));
    } else if (fragment_position.x < 9 * debug_window_width) {
      gua_out_color = vec3(gua_get_position(lookup_texcoord));
    } else if ( fragment_position.x < 10 * debug_window_width) {
      unsigned int nlights = gua_sun_lights_num;
      int bitset_words = ((gua_lights_num - 1) >> 5) + 1;

      ivec2 tile = ivec2(mod(fragment_position.x, debug_window_width ),
                         mod(fragment_position.y, debug_window_height ));

      tile = 5 * tile >> @light_table_tile_power@;

      for (int sl = 0; sl < bitset_words; ++sl) {
        nlights += bitCount(texelFetch(usampler3D(gua_secondary_light_bitset), ivec3(tile, sl), 0).r);
      }
      gua_out_color = vec3(float(nlights) / gua_lights_num);
    }
  }
#endif

  } else if (fragment_position.x < shadow_debug_size && fragment_position.y >= debug_window_height) {

    int shadow_map = (fragment_position.y - debug_window_height) / shadow_debug_size + 1;
    int light_id = -1;

    for (int light_idx = 0; light_idx < gua_lights_num; ++light_idx) {
      if (gua_lights[light_idx].casts_shadow && --shadow_map == 0) {
        light_id = light_idx;
        break;
      }
    }

    if (light_id >= 0) {
      vec2 texcoord = vec2(float(mod(fragment_position.x, shadow_debug_size)) / shadow_debug_size,
                           float(mod(fragment_position.y-debug_window_height, shadow_debug_size)) / shadow_debug_size);

      vec2 lookup_texcoord = texcoord;
    #if @get_enable_multi_view_rendering@
      if(1 == gua_camera_in_multi_view_rendering_mode) {
        lookup_texcoord.x *= 3.0;
        lookup_texcoord.x = clamp(lookup_texcoord.x, 0.0, 1.0);
      }
    #endif

      float intensity = 0.0;
      const int slices = 30;
      for (int i = 0; i < slices; ++i) {
        intensity += texture(sampler2DShadow(gua_lights[light_id].shadow_map), vec3(lookup_texcoord, i * 1.0/slices)).r;
      }

      gua_out_color = vec3(intensity/slices);
      if(lookup_texcoord.x > 0.5) {
        gua_out_color = vec3(1.0, 1.0, 1.0);
      }
    } else {
      discard;
    }

  } else {
    discard;
  }

}

