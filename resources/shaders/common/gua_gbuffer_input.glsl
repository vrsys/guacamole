uniform uvec2 gua_gbuffer_color;
uniform uvec2 gua_gbuffer_pbr;
uniform uvec2 gua_gbuffer_normal;
uniform uvec2 gua_gbuffer_flags;
uniform uvec2 gua_gbuffer_depth;

uniform float gua_texel_width;
uniform float gua_texel_height;

#if @get_enable_multi_view_rendering@

#if @get_enable_hardware_mvr@
    #define LAYER_ID gl_ViewID_OVR
#else
    #define LAYER_ID gl_Layer
#endif
#endif

// quad coords -----------------------------------------------------------------
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}


// helper functions
float gua_scale_unscaled_depth(float unscaled_depth) {
  return unscaled_depth * 2.0 - 1.0;
}

vec3 gua_unproject_depth_to_position(float scaled_depth) {
    vec2 frag_pos = gua_get_quad_coords();
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, scaled_depth, 1.0);

    vec4 h;
#if @get_enable_multi_view_rendering@
    if(0 == LAYER_ID) {
#endif
      h = gua_inverse_projection_view_matrix * screen_space_pos;
#if @get_enable_multi_view_rendering@
    } else {
      h = gua_secondary_inverse_projection_view_matrix * screen_space_pos;      
    }
#endif
    h /= h.w; 
    return h.xyz;
}

// depth -----------------------------------------------------------------------
#if @get_enable_multi_view_rendering@
float gua_get_unscaled_depth(vec2 frag_pos) {
    return texture2DArray(sampler2DArray(gua_gbuffer_depth), vec3(frag_pos, LAYER_ID)).x;
}
#else
float gua_get_unscaled_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x;
}
#endif

float gua_get_unscaled_depth() {
    vec2 frag_pos = gua_get_quad_coords();
    return gua_get_unscaled_depth(gua_get_quad_coords());
}

#if @get_enable_multi_view_rendering@
float gua_get_depth(vec2 frag_pos) {
    return texture2DArray(sampler2DArray(gua_gbuffer_depth), vec3(frag_pos, LAYER_ID)).x * 2.0 - 1.0;
}
#else
float gua_get_depth(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_depth), frag_pos).x * 2.0 - 1.0;
}
#endif
float gua_get_depth() {
    return gua_get_depth(gua_get_quad_coords());
}

#if @get_enable_multi_view_rendering@
float gua_get_depth(sampler2DArray depth_texture, vec2 frag_pos) {
    return texture2DArray(depth_texture, vec3(frag_pos, LAYER_ID) ).x * 2.0 - 1.0;
}
#else
float gua_get_depth(sampler2D depth_texture, vec2 frag_pos) {
    return texture2D(depth_texture, frag_pos).x * 2.0 - 1.0;
}
#endif
float gua_get_depth(sampler2D depth_texture) {
    vec2 frag_pos = gua_get_quad_coords(); 
    return gua_get_depth(gua_get_quad_coords()).x;
}

// position --------------------------------------------------------------------
vec3 gua_get_position(vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(frag_pos), 1.0);

    vec4 h;
#if @get_enable_multi_view_rendering@
    if(0 == LAYER_ID) {
#endif
      h = gua_inverse_projection_view_matrix * screen_space_pos;
#if @get_enable_multi_view_rendering@
    } else {
      h = gua_secondary_inverse_projection_view_matrix * screen_space_pos;      
    }
#endif
    h /= h.w; 
    return h.xyz;
}

vec3 gua_get_position() {
    return gua_get_position(gua_get_quad_coords());
}


#if @get_enable_multi_view_rendering@
vec3 gua_get_position(sampler2DArray depth_texture, vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(depth_texture, frag_pos), 1.0);
    vec4 h;
#if @get_enable_multi_view_rendering@
    if(0 == LAYER_ID) {
#endif
      h = gua_inverse_projection_view_matrix * screen_space_pos;
#if @get_enable_multi_view_rendering@
    } else {
      h = gua_secondary_inverse_projection_view_matrix * screen_space_pos;      
    }
#endif
    h /= h.w;
    return h.xyz;
}

vec3 gua_get_position(sampler2DArray depth_texture) {
    return gua_get_position(depth_texture, gua_get_quad_coords());
} 
#else
vec3 gua_get_position(sampler2D depth_texture, vec2 frag_pos) {
    vec4 screen_space_pos = vec4(frag_pos * 2.0 - 1.0, gua_get_depth(depth_texture, frag_pos), 1.0);
    vec4 h;
#if @get_enable_multi_view_rendering@
    if(0 == LAYER_ID) {
#endif
      h = gua_inverse_projection_view_matrix * screen_space_pos;
#if @get_enable_multi_view_rendering@
    } else {
      h = gua_secondary_inverse_projection_view_matrix * screen_space_pos;      
    }
#endif
    h /= h.w;
    return h.xyz;
}

vec3 gua_get_position(sampler2D depth_texture) {
    return gua_get_position(depth_texture, gua_get_quad_coords());
} 
#endif


// color -----------------------------------------------------------------------
#if @get_enable_multi_view_rendering@
vec3 gua_get_color(vec2 frag_pos) {
    return texture2DArray(sampler2DArray(gua_gbuffer_color), vec3(frag_pos, LAYER_ID) ).rgb;
}
#else
vec3 gua_get_color(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_color), frag_pos).rgb;
}
#endif

vec3 gua_get_color() {
    return gua_get_color(gua_get_quad_coords());
}


// normal -----------------------------------------------------------------------
#if @get_enable_multi_view_rendering@
vec3 gua_get_normal(vec2 frag_pos) {
    return texture2DArray(sampler2DArray(gua_gbuffer_normal), vec3(frag_pos, LAYER_ID) ).rgb * 2 - 1;
}
#else 
vec3 gua_get_normal(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_normal), frag_pos).rgb * 2 - 1;
}
#endif
vec3 gua_get_normal() {
    return gua_get_normal(gua_get_quad_coords());
}


#if @get_enable_multi_view_rendering@
// pbr -------------------------------------------------------------------------
vec3 gua_get_pbr(vec2 frag_pos) {
    return texture2DArray(sampler2DArray(gua_gbuffer_pbr), vec3(frag_pos, LAYER_ID) ).rgb;
}
#else
// pbr -------------------------------------------------------------------------
vec3 gua_get_pbr(vec2 frag_pos) {
    return texture2D(sampler2D(gua_gbuffer_pbr), frag_pos).rgb;
}
#endif

#if @get_enable_multi_view_rendering@
vec3 gua_get_pbr() {
    //return gua_get_pbr(gua_get_quad_coords());
    return texelFetch(sampler2DArray(gua_gbuffer_pbr), ivec3(gl_FragCoord.xy, LAYER_ID), 0).rgb;
}
#else 

vec3 gua_get_pbr() {
    //return gua_get_pbr(gua_get_quad_coords());
    return texelFetch(sampler2D(gua_gbuffer_pbr), ivec2(gl_FragCoord.xy), 0).rgb;
}
#endif

// flags -----------------------------------------------------------------------
#if @get_enable_multi_view_rendering@
uint gua_get_flags(vec2 frag_pos) {
    return uint(texture2DArray(usampler2DArray(gua_gbuffer_flags), vec3(frag_pos, LAYER_ID)).r);
}
#else
uint gua_get_flags(vec2 frag_pos) {
    return uint(texture2D(usampler2D(gua_gbuffer_flags), frag_pos).r);
}
#endif

#if @get_enable_multi_view_rendering@
uint gua_get_flags() {
    return uint(texelFetch(usampler2DArray(gua_gbuffer_flags), ivec3(gl_FragCoord.xy, LAYER_ID), 0).r);
}
#else 
uint gua_get_flags() {
    return uint(texelFetch(usampler2D(gua_gbuffer_flags), ivec2(gl_FragCoord.xy), 0).r);
}
#endif