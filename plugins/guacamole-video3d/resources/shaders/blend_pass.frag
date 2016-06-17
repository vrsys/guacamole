@include "shaders/common/header.glsl"
@include "shaders/common/gua_fragment_shader_input.glsl"
@include "shaders/common/gua_camera_uniforms.glsl"
@include "shaders/common/pack_vec3.glsl"

@material_uniforms@

///////////////////////////////////////////////////////////////////////////////
// video3D uniforms
///////////////////////////////////////////////////////////////////////////////
#define MAX_VIEWS 8
uniform int   numlayers;
uniform float epsilon;
uniform int   overwrite_normal;
uniform vec3  o_normal;

uniform sampler2DArray depth_texture;
uniform sampler2DArray quality_texture;
uniform sampler2DArray video_color_texture;

@include "shaders/common/gua_fragment_shader_output.glsl"
@include "shaders/common/gua_global_variable_declaration.glsl"

@material_method_declarations_frag@

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() {

  @material_input@

  @include "shaders/common/gua_global_variable_assignment.glsl"

  vec3  output_color  = vec3(0.0);
  float output_depth  = 1.0f;
  vec3  output_normal = vec3(0.0);

  vec3 coords = vec3(gua_texcoords, 0.0);

  const float maxdist = 1.0 / 0.0; // infinity 1000.0;
  vec4 layer_contributions[MAX_VIEWS];

  float mindist = maxdist;
  float ogldepth = 1.0;

  // find minimum distances;
  for(int l = 0; l  < numlayers && l < MAX_VIEWS;++l)
  {
    coords.z = float(l);
    const float depth         = texture(depth_texture, coords).r;
    if(depth < 1.0){ // found kinect surface in layer
      vec4 layer_contribution = texture(quality_texture, coords);
      vec3 normal             = unpack_vec3(layer_contribution.b);
      vec4 p_os = gua_inverse_projection_matrix * vec4(gua_texcoords.xy * 2.0f - vec2(1.0), depth*2.0f - 1.0f, 1.0);
      p_os = p_os / p_os.w;
      layer_contribution.b = length(p_os.xyz);
      const float dist = layer_contribution.b;
      layer_contributions[l] = layer_contribution;

      if(dist < mindist){ // track minimum distance to eye
      	mindist = dist;
      	ogldepth = depth;
        output_normal = normal;
      }
    }
    else{
      layer_contributions[l] = vec4(1.0,1.0,maxdist,1.0);
    }
  }

  int accum = 0;
  int best_layer = 0;
  float best_quality = 0.0;
  if(mindist < maxdist) {  // we found at least one kinect surface
    vec4 finalcol = vec4(0.0);

    for(int l = 0; l  < numlayers && l < MAX_VIEWS;++l){
      if( abs(layer_contributions[l].b - mindist) < epsilon){
      	++accum;
      	vec4 layer_contribution = layer_contributions[l];
      	vec3 color = texture( video_color_texture, vec3(layer_contribution.xy,float(l))).rgb;

      	finalcol.rgb += color * layer_contribution.a;
      	finalcol.a   += layer_contribution.a;

      	if(layer_contribution.a > best_quality){ // track best layer
      	  best_quality = layer_contribution.a;
      	  best_layer = l;
      	}
      }
    }
    finalcol.rgba = finalcol.rgba/finalcol.a;
    output_color  = finalcol.rgb;
    output_depth  = ogldepth;
  }
  else {
    discard;
  }

  // if(overwrite_normal > 0){
  //   output_normal = o_normal;
  // }

  gua_normal = output_normal;
  gua_color = output_color;
#if 0
  if(layer_contributions[0].a < 0.999999)
    gua_color.r = layer_contributions[0].a;
  else
    gua_color.r = 0;
  if(layer_contributions[1].a < 0.999999)
    gua_color.g = layer_contributions[1].a;
  else
    gua_color.g = 0;
  if(layer_contributions[2].a < 0.999999)
    gua_color.b = layer_contributions[2].a;
  else
    gua_color.b = 0;

#endif  
#if 0
  gua_color = vec3(layer_contributions[0].a);
  if(gua_color.r > 0.999999)
    discard;
#endif

  gl_FragDepth = output_depth;

  @material_method_calls_frag@

  @include "shaders/common/gua_write_gbuffer.glsl"
}
