@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in vec3 gua_in_position;
#if 0
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;
#endif

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// video3d uniforms
///////////////////////////////////////////////////////////////////////////////
uniform mat4  image_d_to_eye_d;
uniform mat4  eye_d_to_world;
uniform mat4  eye_d_to_eye_rgb;
uniform mat4  eye_rgb_to_image_rgb;

uniform sampler2DArray depth_video3d_texture;
uniform sampler3D cv_xyz;
uniform sampler3D cv_uv;
uniform float cv_min_d;
uniform float cv_max_d;
uniform vec2 tex_size_inv;
uniform int layer;

///////////////////////////////////////////////////////////////////////////////
// outputs
///////////////////////////////////////////////////////////////////////////////
out VertexData {
    vec2 texture_coord;
    vec3 pos_es;
    vec3 pos_d;
    vec3 pos_ws;
    float depth;
} VertexOut;



///////////////////////////////////////////////////////////////////////////////
// bilateral filter
///////////////////////////////////////////////////////////////////////////////

int kernel_size = 5; // in pixel
int kernel_end = kernel_size + 1;


float dist_space_max_inv = 1.0/float(kernel_size);
float computeGaussSpace(float dist_space){
  float gauss_coord = dist_space * dist_space_max_inv;
  return 1.0 - gauss_coord;//texture2D(gauss,vec2(gauss_coord,0.5)).r;
}

float dist_range_max = 0.05; // in meter
float dist_range_max_inv = 1.0/dist_range_max;
float computeGaussRange(float dist_range){
  float gauss_coord = min(dist_range, dist_range_max) * dist_range_max_inv;
  return 1.0 - gauss_coord;//texture2D(gauss,vec2(gauss_coord,0.5)).r;
}


float bilateral_filter(){
  vec3 coords = vec3(gua_in_position.xy,layer);
  float depth = texture2DArray(depth_video3d_texture, coords).r;
  if(depth < 0.01){
     return 0.0;
  }

#if 1
  // weiter hinten liegende samples haben einen groesseren range
  float max_depth = 7.0;
  float d_dmax = depth/max_depth;

  dist_range_max = 0.5 * d_dmax;
  if(dist_range_max > 0.0)
    dist_range_max_inv = 1.0/dist_range_max;
  else
    dist_range_max_inv = 20.0;
#endif

  float depth_bf = 0.0;

  float w = 0.0;
  float w_range = 0.0;
  for(int y = -kernel_size; y < kernel_end; ++y){
    for(int x = -kernel_size; x < kernel_end; ++x){
      vec3 coords_s = vec3(coords.s + float(x) * tex_size_inv.x, coords.t + float(y) * tex_size_inv.y, float(layer));
      
      float depth_s = texture2DArray(depth_video3d_texture, coords_s).r;

      float gauss_space = computeGaussSpace(length(vec2(x,y)));
      float gauss_range = computeGaussRange(abs(depth_s - depth));
      float w_s = gauss_space * gauss_range;
      depth_bf += w_s * depth_s;
      w += w_s;
      w_range += gauss_range;
    }
  }

  float filtered_depth = 0.0;
  if(w > 0.0)
    filtered_depth = depth_bf/w;
  else
    filtered_depth = 0.0;

#if 1
  if(w_range < 70.1)
    filtered_depth = 0.0;
#endif

  return filtered_depth;
}



///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() 
{
#if 0
  float depth             = texture2DArray(depth_video3d_texture, vec3(gua_in_position.xy, layer)).r;
#else
  float depth             = bilateral_filter();
#endif

  vec4 POS_d              = depth * image_d_to_eye_d * vec4(gua_in_position.xy, depth, 1.0);
  POS_d.z                 = depth;
  POS_d.w                 = 1.0;
  
  vec4 POS_rgb            = eye_d_to_eye_rgb * POS_d;
  vec4 POS_ws             = eye_d_to_world * POS_d;

  // lookup from calibvolume
  float d_idx = (depth - cv_min_d)/(cv_max_d - cv_min_d);
  POS_ws = vec4(texture(cv_xyz, vec3(gua_in_position.xy, d_idx)).rgb, 1.0);

  VertexOut.pos_d         = POS_d.xyz;
  VertexOut.pos_ws        = POS_ws.xyz;
  VertexOut.pos_es        = (gua_view_matrix * gua_model_matrix * POS_ws).xyz;

  VertexOut.texture_coord = texture(cv_uv,  vec3(gua_in_position.xy, d_idx)).rg;
  //VertexOut.texture_coord = (eye_rgb_to_image_rgb * vec4( (POS_rgb.xy/POS_rgb.z) ,0.0, 1.0)).xy;


  VertexOut.depth         = depth;
  
  gl_Position             = gua_projection_matrix * gua_view_matrix * gua_model_matrix * POS_ws;
}

