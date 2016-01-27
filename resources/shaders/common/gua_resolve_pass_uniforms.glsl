uniform int     gua_background_mode;
uniform vec3    gua_background_color;
uniform uvec2   gua_background_texture;
uniform uvec2   gua_alternative_background_texture;
uniform float   gua_background_texture_blend_factor;

uniform int     gua_environment_lighting_mode;
uniform uvec2   gua_environment_lighting_texture;
uniform uvec2   gua_alternative_environment_lighting_texture;
uniform float   gua_environment_lighting_texture_blend_factor;
uniform vec3    gua_environment_lighting_color;
uniform float   gua_horizon_fade;

uniform bool    gua_ssao_enable;
uniform float   gua_ssao_radius;
uniform float   gua_ssao_intensity;
uniform float   gua_ssao_falloff;

uniform bool    gua_screen_space_shadows_enable = false;
uniform float   gua_screen_space_shadows_radius = 1.0;
uniform float   gua_screen_space_shadows_max_radius_px = 200.0;
uniform float   gua_screen_space_shadows_intensity = 0.8;

uniform uvec2   gua_noise_tex;

uniform bool    gua_enable_fog = false;
uniform float   gua_fog_start  = 1.0;
uniform float   gua_fog_end    = 100.0;

uniform float   gua_vignette_coverage;
uniform float   gua_vignette_softness;
uniform vec4    gua_vignette_color;

uniform float   gua_tone_mapping_exposure = 1.0;
