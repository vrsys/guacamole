uniform uvec2 gua_gbuffer_color;
uniform uvec2 gua_gbuffer_pbr;
uniform uvec2 gua_gbuffer_normal;
uniform uvec2 gua_gbuffer_flags;
uniform uvec2 gua_gbuffer_depth;

uniform float gua_texel_width;
uniform float gua_texel_height;

// quad coords -----------------------------------------------------------------
vec2 gua_get_quad_coords() {

  float frag_coord_x = gl_FragCoord.x;


  return vec2(frag_coord_x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}


// helper functions
float gua_scale_unscaled_depth(float unscaled_depth) {
  return unscaled_depth * 2.0 - 1.0;
}

vec3 gua_unproject_depth_to_position(float scaled_depth) {
    vec2 frag_pos = gua_get_quad_coords();

    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, scaled_depth, 1.0);

    #if @get_enable_multi_view_rendering@
      if(! (0 == gua_camera_in_multi_view_rendering_mode) ) {
        if(0 == gl_ViewportIndex) {
          screen_space_pos.x = screen_space_pos.x * 2 + 1;
        } else {
          screen_space_pos.x = screen_space_pos.x * 2 - 1;
        }
      }
      //screen_space_pos.x *= 2.0;
      //screen_space_pos.x -= 1.0;
    #endif

    vec4 h = vec4(0.0, 0.0, 0.0, 1.0);
    #if @get_enable_multi_view_rendering@
      if(1 == gl_ViewportIndex) {
        h = gua_secondary_inverse_projection_view_matrix * screen_space_pos;
      } else {
    #endif
      h = gua_inverse_projection_view_matrix * screen_space_pos;
    #if @get_enable_multi_view_rendering@
      }
    #endif
    h /= h.w; 
    return h.xyz;
}

// depth -----------------------------------------------------------------------

float gua_get_unscaled_depth() {
    vec2 frag_pos = gua_get_quad_coords();
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x;
}

float gua_get_unscaled_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x;
}

float gua_get_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth() {
    vec2 frag_pos = gua_get_quad_coords();
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture, vec2 frag_pos) {
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}

float gua_get_depth(sampler2D depth_texture) {
    vec2 frag_pos = gua_get_quad_coords(); 
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}

// position --------------------------------------------------------------------
vec3 gua_get_position(vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(frag_pos), 1.0);
    vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
    h /= h.w; 
    return h.xyz;
}

vec3 gua_get_position() {
    return gua_get_position(gua_get_quad_coords());
}

vec3 gua_get_position(sampler2D depth_texture, vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(depth_texture, frag_pos), 1.0);
    vec4 h = gua_inverse_projection_view_matrix * screen_space_pos;
    h /= h.w;
    return h.xyz;
}

vec3 gua_get_position(sampler2D depth_texture) {
    return gua_get_position(depth_texture, gua_get_quad_coords());
} 

// color -----------------------------------------------------------------------
vec3 gua_get_color(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_color), frag_pos).rgb;
}

vec3 gua_get_color() {
    return gua_get_color(gua_get_quad_coords());
}

// normal ----------------------------------------------------------------------
vec3 gua_get_normal(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_normal), frag_pos).rgb * 2 - 1;
}

vec3 gua_get_normal() {
    return gua_get_normal(gua_get_quad_coords());
}

// pbr -------------------------------------------------------------------------
vec3 gua_get_pbr(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_pbr), frag_pos).rgb;
}

vec3 gua_get_pbr() {
    //return gua_get_pbr(gua_get_quad_coords());
    return texelFetch(sampler2D(gua_gbuffer_pbr), ivec2(gl_FragCoord.xy), 0).rgb;
}

// flags -----------------------------------------------------------------------
uint gua_get_flags(vec2 frag_pos) {
    return uint(texture2D(usampler2D(gua_gbuffer_flags), frag_pos).r);
}

uint gua_get_flags() {
    return uint(texelFetch(usampler2D(gua_gbuffer_flags), ivec2(gl_FragCoord.xy), 0).r);
}
