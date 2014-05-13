@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec3 gua_position_varying;
in vec2 gua_quad_coords;

@input_definition


///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

@uniform_definition


///////////////////////////////////////////////////////////////////////////////
// video3D uniforms
///////////////////////////////////////////////////////////////////////////////
#define MAX_VIEWS 8
uniform int   numlayers;
uniform float epsilon;
uniform int   using_default_video_material;

uniform sampler2DArray depth_texture;
uniform sampler2DArray quality_texture;
uniform sampler2DArray video_color_texture;


///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
@output_definition


///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/pack_vec3.glsl"

vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

@include "shaders/uber_shaders/common/get_sampler_casts.glsl"

uint gua_get_material_id() {
  return gua_uint_gbuffer_varying_0.x;
}

vec3 gua_get_position() {
  return gua_position_varying;
}

// material specific methods
@material_methods


///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  vec3  output_color  = vec3(0.0);
  float output_depth  = 1.0f;
  vec3  output_normal = vec3(0.0);

  vec3 coords = vec3(gua_quad_coords, 0.0);

  vec4 color_contributions[MAX_VIEWS];

  float maxdist = 1000.0;
  float mindist = maxdist;
  float ogldepth = 1.0;

  // find minimum distances;
  for(int l = 0; l  < numlayers && l < MAX_VIEWS;++l)
  {
    coords.z = float(l);
    vec4 color_contribution = texture2DArray( quality_texture, coords);
    float depth             = texture2DArray( depth_texture, coords).r;
    vec3 normal             = unpack_vec3(color_contribution.b);
#if 0
#else
    vec4 p_os = gua_inverse_projection_matrix * vec4(gua_quad_coords.xy * 2.0f - vec2(1.0), depth*2.0f - 1.0f, 1.0);
    p_os = p_os / p_os.w;
    color_contribution.b = length(p_os.xyz);
#endif

    float dist = color_contribution.b;

    if(dist < maxdist && depth < 1.0)
    {
      color_contributions[l] = color_contribution;
      if(dist < mindist){
      	mindist = dist;
      	ogldepth = depth;
        output_normal = normal;
      }
    } else {
      color_contributions[l] = vec4(1.0,1.0,maxdist,1.0);
    }
  }

  int accum = 0;
  if(mindist < maxdist)  // we found at least one surface
  {
    vec4 finalcol = vec4(0.0);

    for(int l = 0; l  < numlayers && l < MAX_VIEWS;++l)
    {
      if( abs(color_contributions[l].b - mindist) < epsilon)
      {
	      ++accum;
	      vec4 color_contribution = color_contributions[l];
	      vec4 color = texture2DArray( video_color_texture, vec3(color_contribution.xy,float(l)));
	      finalcol.rgb += color.rgb * color_contribution.a;
	      finalcol.a   += color_contribution.a;
      }
    }
    if(finalcol.a > 0.0)
    {
      finalcol.rgba = finalcol.rgba/finalcol.a;
      output_depth  = ogldepth;
      output_color  = finalcol.rgb;
    } else {
      discard;
    }
  } else {
    discard;
  }

  // big switch, one case for each material
  @material_switch

  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;

  if ( using_default_video_material != 0 )
  {
    @apply_video3d_color
    @apply_video3d_normal
  }

  gl_FragDepth = output_depth;
}
