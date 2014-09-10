@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// video3d uniforms
///////////////////////////////////////////////////////////////////////////////
uniform int layer;
uniform int bbxclip;
uniform vec3 bbx_min;// = vec3(-1.5,-0.1, -1.0);
uniform vec3 bbx_max;// = vec3( 1.5,2.2,   1.5);

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
in vec2  texture_coord;
in vec3  pos_es;
in vec3  pos_d;
in vec3  pos_ws;
in float depth;
in vec3  normal_es;

///////////////////////////////////////////////////////////////////////////////
// output
///////////////////////////////////////////////////////////////////////////////
layout (location=0) out vec4 out_color;

///////////////////////////////////////////////////////////////////////////////
// methods
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/pack_vec3.glsl"

// methods 

bool clip(vec3 p){
  if(p.x < bbx_min.x ||
     p.y < bbx_min.y ||
     p.z < bbx_min.z ||
     p.x > bbx_max.x ||
     p.y > bbx_max.y ||
     p.z > bbx_max.z){
    return true;
  }
  return false;
}

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() 
{
#if 1
  if(clip(pos_ws) && bbxclip > 0)
    discard;
#endif

  // caclulate adhoc normal from ddepth
  vec3 a_d = dFdx(pos_d);
  vec3 b_d = -dFdy(pos_d);
  vec3 normal_d = normalize(cross(b_d,a_d));
  float d_angle = dot(normalize(-pos_d),normal_d);

#if 1
  // back face culling
  if(d_angle < 0.075) {
     discard;
  }
#endif

#if 1
   // to cull away borders of the rgb camera view
   if(texture_coord.s > 0.975 || texture_coord.s < 0.025 ||
      texture_coord.t > 0.975 || texture_coord.t < 0.025) {
        discard;
   }
#endif

  vec3 pos_d_to_depth_camera = normalize(pos_d);
  pos_d_to_depth_camera.x = pos_d_to_depth_camera.x;
  pos_d_to_depth_camera.y = pos_d_to_depth_camera.y;
  pos_d_to_depth_camera.z = -pos_d_to_depth_camera.z;

  float cosphi = dot(normal_d, pos_d_to_depth_camera);
  float depth_squared = (depth * depth);
  float quality = 1000.0;

  if(depth_squared > 0.0)
    quality = cosphi / depth_squared;
  else
    quality = 0.0;

  quality = quality * quality;

  float dist_es = length(pos_es);

  float packed_normal = pack_vec3(normalize(normal_es));

#if 0
  out_color = vec4(texture_coord, dist_es, quality);
#else
  out_color = vec4(texture_coord, packed_normal, quality);
#endif
}
