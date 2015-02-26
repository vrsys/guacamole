
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


vec3 object_space_random() {

  const int A = 123417123;
  const int B = 621;
  const int C = 721191;

  vec4 object_space = inverse(gua_model_matrix) * vec4(gua_get_position(), 1.0);

  int x = int(object_space.x * float(A));
  int y = int(object_space.y * float(A));
  int z = int(object_space.z * float(A));

  int random_x = (A * (( x % B ) * C * A)%B) % B;
  int random_y = (A * (( y % B ) * C * A)%B) % B;
  int random_z = (A * (( z % B ) * C * A)%B) % B;

  return vec3(float(random_x)/float(B), float(random_y)/float(B), float(random_z)/float(B));
}

float compute_ssao () 
{
  vec2 texcoords = gua_get_quad_coords();

  vec3 n = gua_get_normal();
  vec3 p = gua_get_position();

  //randomization texture
  vec2 fres = vec2(1.0/(64.0*gua_texel_width)*5,1.0/(64.0*gua_texel_height)*5);
  //vec3 random = texture2D(sampler2D(gua_noise_tex), texcoords.st*fres.xy).xyz;
  vec3 random = object_space_random();

  random = (random-vec3(0.5)) * gua_ssao_radius;

  //initialize variables
  float ao = 0.0;
  float incx = gua_texel_width * gua_ssao_radius;
  float incy = gua_texel_height * gua_ssao_radius;
  float pw = incx;
  float ph = incy;
  float cdepth = max(gua_get_depth(), 0.01);

  //3 rounds of 8 samples each.
  for(float i=0.0; i<4.0; ++i) {
    float npw = (pw+0.001*random.x)/cdepth;
    float nph = (ph+0.001*random.y)/cdepth;

    vec3 ddiff =  gua_get_position(texcoords+vec2(npw,nph))-p;
    vec3 ddiff2 = gua_get_position(texcoords+vec2(npw,-nph))-p;
    vec3 ddiff3 = gua_get_position(texcoords+vec2(-npw,nph))-p;
    vec3 ddiff4 = gua_get_position(texcoords+vec2(-npw,-nph))-p;
    vec3 ddiff5 = gua_get_position(texcoords+vec2(0,nph))-p;
    vec3 ddiff6 = gua_get_position(texcoords+vec2(0,-nph))-p;
    vec3 ddiff7 = gua_get_position(texcoords+vec2(npw,0))-p;
    vec3 ddiff8 = gua_get_position(texcoords+vec2(-npw,0))-p;

    ao+= gua_ssao_aoFF(ddiff, n, npw, nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff2, n, npw, -nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff3, n, -npw, nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff4, n, -npw, -nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff5, n, 0, nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff6, n, 0, -nph, texcoords);
    ao+= gua_ssao_aoFF(ddiff7, n, npw, 0, texcoords);
    ao+= gua_ssao_aoFF(ddiff8, n, -npw, 0, texcoords);

    pw += incx;
    ph += incy;
  }

  ao/=32.0;

  return ao * gua_ssao_intensity;
}
