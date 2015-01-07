
float saturate(float x) { return clamp(x, 0.0f, 1.0f); }

// diffuse
vec3 lambert(vec3 col)
{
  return col / 3.14159265;
}

// for microsoft BRDFs (microsurface normal m == h the half vector)
// F_schlick(F0,l,h) = F0 + (1 - F0)*(1-dot(l,h))^5

// From s2013_pbs_rad_notes.pdf
// ===============================================================================
// Calculates the Fresnel factor using Schlick’s approximation
// ===============================================================================
vec3 Fresnel(vec3 specAlbedo, vec3 h, vec3 l)
{
  float lDotH = saturate(dot(l, h));
  //return specAlbedo + (1.0f - specAlbedo) * pow((1.0f - lDotH), 5.0f);
  // see http://seblagarde.wordpress.com/2012/06/03/spherical-gaussien-approximation-for-blinn-phong-phong-and-fresnel/
  // pow(1-lDotH, 5) = exp2((-5.55473 * ldotH - 6.98316) * ldotH)
  return specAlbedo + ( saturate( 50.0 * specAlbedo.g ) - specAlbedo ) * exp2( (-5.55473 * lDotH - 6.98316) * lDotH );
}

// ===============================================================================
// Helper for computing the GGX visibility term
// ===============================================================================
float GGX_V1(in float m2, in float nDotX)
{
  return 1.0f / (nDotX + sqrt(m2 + (1 - m2) * nDotX * nDotX));
}

// ===============================================================================
// Computes the specular term using a GGX microfacet distribution, with a
// matching geometry factor and visibility term. m is roughness, n is the surface
// normal, h is the half vector, l is the direction to the light source, and
// specAlbedo is the RGB specular albedo
// ===============================================================================
float GGX_Specular(float m, vec3 n, vec3 h, vec3 v, vec3 l)
{
  float nDotL = saturate(dot(n, l));
  if(nDotL <= 0.0f)
    return 0.0f;
  float nDotH = saturate(dot(n, h));
  float nDotV = max(dot(n, v), 0.0001f);
  float nDotH2 = nDotH * nDotH;
  float m2 = m * m;
  // Calculate the distribution term
  float d = m2 / (Pi * pow(nDotH * nDotH * (m2 - 1) + 1, 2.0f));
  // Calculate the matching visibility term
  float v1i = GGX_V1(m2, nDotL);
  float v1o = GGX_V1(m2, nDotV);
  float vis = v1i * v1o;
  // Put it all together
  return d * vis;
}

// ===============================================================================
// Helper for computing the Beckmann geometry term
//
// ===============================================================================
// float Beckmann_G1(float m, float nDotX)
// {
//   float nDotX2 = nDotX * nDotX;
//   float tanTheta = sqrt((1 - nDotX2) / nDotX2);
//   float a = 1.0f / (m * tanTheta);
//   float a2 = a * a;
//   float g = 1.0f;
//   if(a < 1.6f)
//     g *= (3.535f * a + 2.181f * a2) / (1.0f + 2.276f * a + 2.577f * a2);
//   return g;
// }

// ===============================================================================
// Computes the specular term using a Beckmann microfacet distribution, with a
// matching geometry factor and visibility term. m is roughness, n is the surface
// normal, h is the half vector, l is the direction to the light source, and
// specAlbedo is the RGB specular albedo
// ===============================================================================
// assumes that m != 0
// vec3 Beckmann_Specular(float m, vec3 n, vec3 h, vec3 v, vec3 l, vec3 specAlbedo)
// {
//   float nDotL = saturate(dot(n, l));
//   if(nDotL <= 0.0f)
//     return vec3(0.0f);
//   float nDotH = saturate(dot(n, h));
//   float nDotV = max(dot(n, v), 0.0001f);
//   float nDotH2 = nDotH * nDotH;
//   float nDotH4 = nDotH2 * nDotH2;
//   float m2 = m * m;

//   // Calculate the distribution term -- uses normal, halfvector h, and roughness
//   // warum nicht nDotH2 - 1 ?
//   float tanTheta2 = (1 - nDotH2) / nDotH2;
//   float expTerm = exp(-tanTheta2 / m2);
//   // warum nicht m^4 ?
//   float D = expTerm / (Pi * m2 * nDotH4);
//   // Calculate the matching geometric term
//   float g1i = Beckmann_G1(m, nDotL);

//   float g1o = Beckmann_G1(m, nDotV);
//   float G = g1i * g1o;
//   // Calculate the fresnel term
//   //float f = Fresnel(specAlbedo, h, l);
//   vec3 F = Fresnel(specAlbedo, h, l);
//   // Put it all together
//   return D * G * F * (1.0f / (4.0f * nDotL * nDotV));
// }

// // End - From s2013_pbs_rad_notes.pdf

// // http://renderwonk.com/publications/s2010-shading-course/hoffman/s2010_physically_based_shading_hoffman_a_notes.pdf
// // schlick , use h for n with microfacets
// // f0 - reflection coefficient for light incoming parallel to the normal
// vec3 F_schlick(vec3 f0, vec3 L, vec3 N)
// {
//   return f0 + (1.0 - f0) * pow(1.0 - saturate(dot(L, N))  , 5);
// }

// float F_schlick( float LdotH )
// {
//   //float x  = clamp( 1 - LdotH, 0.0, 1.0 );
//   float x  = 1.0 - saturate(LdotH);
//   float x2 = x * x;
//   return ( x2 * x2 * x );
// }

// float F_schlick(vec3 L, vec3 N, float eta)
// {
//   float sqr_f0 = (1-eta)*(1+eta);
//   float f0 = sqr_f0 * sqr_f0;
//   return f0 + (1.0 - f0) * pow(1.0 - saturate(dot(L, N))  , 5);
// }

// vec3 F_schlick(float n1, float n2, float costheta)
// {
//   float sqr_f0 = (n1 - n2)/(n1+n2);
//   float f0 = sqr_f0*sqr_f0;
//   return vec3(f0 + (1.0 - f0) * pow(1.0 - saturate(costheta)  , 5));
// }

// vec3 fresnelSchlickWithRoughness(vec3 c_spec,vec3 E,vec3 N,float gloss)
// {
//   return c_spec + (max(vec3(gloss), c_spec) - c_spec) * pow(1 - saturate(dot(E, N)), 5);
// }
