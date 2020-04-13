
float gua_ssao_aoFF(in vec3 ddiff,in vec3 cnorm, in float c1, in float c2, in vec2 texcoord) {
  float rd = length(ddiff);
  vec3 vv = ddiff / rd;
  return (1.0-clamp(dot(gua_get_normal(texcoord+vec2(c1,c2)),-vv),0.0,1.0)) *
        clamp(dot( cnorm,vv ),0.0,1.0) * (1.0 - 1.0/sqrt(1.0/(rd*rd*gua_ssao_falloff) + 1.0));
}

float gua_ssao_giFF(in vec3 ddiff,in vec3 cnorm, in float c1, in float c2, in vec2 texcoord, in sampler2D gnormals) {
  float rd = length(ddiff);
  vec3 vv = ddiff / rd;
  return 1.0*clamp(dot(gua_get_normal(texcoord+vec2(c1,c2)),-vv),0.0,1.0)*clamp(dot( cnorm,vv ),0.0,1.0)/(rd*rd+1.0);
}

const vec3 object_space_hardcoded_random = ( (123417123 * ((    ivec3(vec3(1.0) * float(123417123) )   % 621 ) * 721191 * 123417123) % 621) % 621 ) / 621.0;

vec3 object_space_random() {

  const int A = 123417123;
  const int B = 621;
  const int C = 721191;

  // vec4 object_space = inverse(gua_model_matrix) * vec4(gua_get_position(), 1.0);
  vec4 object_space = vec4(1.0);

  ivec3 xyz = ivec3(object_space.xyz * float(A));
  //int x = int(object_space.x * float(A));
  //int y = int(object_space.y * float(A));
  //int z = int(object_space.z * float(A));

  ivec3 random_xyz = (A * (( xyz % B ) * C * A)%B) % B;

  //int random_x = (A * (( x % B ) * C * A)%B) % B;
  //int random_y = (A * (( y % B ) * C * A)%B) % B;
  //int random_z = (A * (( z % B ) * C * A)%B) % B;

  return vec3(random_xyz) / float(B);
}


float compute_ssao (in vec3 normal, in vec3 position, in float in_depth) 
{
  vec2 texcoords = gua_get_quad_coords_eye_separated() ;

  //vec3 n = normal;
  //vec3 p = position;

  //randomization texture
  //vec2 fres = 5.0 / (64.0 *vec2(gua_texel_width, gua_texel_height) );  // vec2(1.0/(64.0*gua_texel_width)*5,1.0/(64.0*gua_texel_height)*5);
  //vec3 random = texture2D(sampler2D(gua_noise_tex), texcoords.st*fres.xy).xyz;
  vec3 random = object_space_hardcoded_random; //object_space_random();//

  random = (random-vec3(0.5)) * gua_ssao_radius;

  //initialize variables
  float ao = 0.0;
  vec2 incxy = vec2(gua_texel_width, gua_texel_height) * gua_ssao_radius;
  
  vec2 pwh = incxy;
  float cdepth = max(in_depth, 0.1);



  //vec3 ssao_sampling_vals[8];

  //3 rounds of 8 samples each.
  for(uint ssao_it = 0; ssao_it < 4; ++ssao_it) {
    vec2 npwh = (pwh+0.001*random.xy)/cdepth;
    //float nph = (pwh.y+0.001*random.y)/cdepth;

    vec3 ddiff =  gua_get_position(texcoords+vec2(npwh))-position;
    vec3 ddiff2 = gua_get_position(texcoords+vec2(npwh.x,-npwh.y))-position;
    vec3 ddiff3 = gua_get_position(texcoords+vec2(-npwh.x,npwh.y))-position;
    vec3 ddiff4 = gua_get_position(texcoords+vec2(-npwh.x,-npwh.y))-position;
    vec3 ddiff5 = gua_get_position(texcoords+vec2(0,npwh.y))-position;
    vec3 ddiff6 = gua_get_position(texcoords+vec2(0,-npwh.y))-position;
    vec3 ddiff7 = gua_get_position(texcoords+vec2(npwh.x,0))-position;
    vec3 ddiff8 = gua_get_position(texcoords+vec2(-npwh.x,0))-position;

    ao += gua_ssao_aoFF(ddiff, normal, npwh.x, npwh.y, texcoords);
    ao += gua_ssao_aoFF(ddiff2, normal, npwh.x, -npwh.y, texcoords);
    ao += gua_ssao_aoFF(ddiff3, normal, -npwh.x, npwh.y, texcoords);
    ao += gua_ssao_aoFF(ddiff4, normal, -npwh.x, -npwh.y, texcoords);
    ao += gua_ssao_aoFF(ddiff5, normal, 0, npwh.y, texcoords);
    ao += gua_ssao_aoFF(ddiff6, normal, 0, -npwh.y, texcoords);
    ao += gua_ssao_aoFF(ddiff7, normal, npwh.x, 0, texcoords);
    ao += gua_ssao_aoFF(ddiff8, normal, -npwh.x, 0, texcoords);
  

    pwh += incxy;
  }

  ao *= 0.03125; // <- 1/32

  return clamp(ao * gua_ssao_intensity, 0.0, 1.0);
}
