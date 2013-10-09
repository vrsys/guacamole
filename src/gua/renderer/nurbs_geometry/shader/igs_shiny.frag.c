#if 0

#version 400 core

#extension GL_NV_gpu_shader5 : enable

uniform sampler2D mat;
uniform sampler2D env;

uniform float emit;

layout(location=0) out vec3 out_color;
layout(location=1) out vec3 out_normal;
layout(location=2) out vec3 out_specular_emit;
layout(location=3) out vec3 out_position;

in vec3 	gPosition;
flat in float 	gIndex;
in vec2 	gTessCoord;
in vec4 	gNormal;

flat in vec2 gTemp;

uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 normal_matrix;

uniform samplerBuffer attribute_texture;

uniform samplerBuffer trimdata;
uniform samplerBuffer celldata;
uniform samplerBuffer curvelist;
uniform samplerBuffer curvedata;

/****************************************************
* surface error in object space
****************************************************/
#define TRIM_ERROR_TOLERANCE 0.00001

precision highp float;

void evaluateCurve(in samplerBuffer curvedata, in int index, in int order, in float t, out vec4 p);
bool binary_search ( in samplerBuffer buf, in float value, in int id, in int intervals, inout vec4 result);
void bisect(in samplerBuffer curvedata, in vec2 uv, in vec4 curve, in vec4 startend, inout int intersections);
void trim(in samplerBuffer trimdata,  in samplerBuffer celldata, in samplerBuffer curvelist, in samplerBuffer curvedata, in int trim_index, in bool trimtype, in vec2 uv);

const float invpi = 0.31830988618379067153776752674503;

float fresnel(in vec3 v, in vec3 n)
{
    const float f = 0.3;
    float ret = (1.0 - dot(v, n));
    ret = f + (1.0 - f) * pow(ret, 15.0);
    return min(ret, 1.0);
}

vec2 env_long_lat(in vec3 v)
{
    vec2 a_xz = normalize(v.xz);
    vec2 a_yz = normalize(v.yz);

    return vec2(0.5 * (1.0 + invpi * atan(a_xz.x, -a_xz.y)),
                acos(-v.y) * invpi);
}

void main()
{
	vec4 _domaindata = texelFetch(attribute_texture, int(gIndex) * 5 + 1);

	float nurbs_u = _domaindata[0] + gTessCoord[0] * (_domaindata[1] - _domaindata[0]);
	float nurbs_v = _domaindata[2] + gTessCoord[1] * (_domaindata[3] - _domaindata[2]);

	int trim_index = int(texelFetch(attribute_texture, int(gIndex) * 5).w);

	trim(trimdata, celldata, curvelist, curvedata, trim_index, true, vec2(nurbs_u, nurbs_v));

        out_normal = normalize(gNormal.xyz);

	vec3 vvec = normalize(gPosition);
	vec3 refl = reflect(vvec, gNormal.xyz);

        out_color = vec3(gTessCoord, 0.0);//mix(texture2D(mat, gTessCoord).xyz * 0.2, texture2D(env, env_long_lat(refl)).xyz * 0.5, emit * fresnel(-vvec, normalize(gNormal.xyz)));//vec3(gTessCoord, 0.0);//texture2D(mat, gTessCoord).xyz; vec3(model_matrix * vec4(gPosition, 1.0)); //gNormal.xyz; //mix(texture2D(mat, gTessCoord).xyz * 0.2, texture2D(env, env_long_lat(refl)).xyz * 0.5, emit * fresnel(-vvec, normalize(gNormal.xyz)));//vec3(gTessCoord, 0.0);//texture2D(mat, gTessCoord).xyz;

	out_position = vec3(model_matrix * vec4(gPosition, 1.0));

        out_specular_emit = vec3(1, emit, 0.5);
}

/*******************************************************************************
 * Evaluate Curve using modificated horner algorithm in Bernstein basis        *
 *   - points are supposed to be in hyperspace : [wx, wy, w]                   *
 *   - curvedata[index] is the first point of curve                            *
 *   - t is the parameter the curve is to be evaluated for                     *
 ******************************************************************************/
void
evaluateCurve(in samplerBuffer curvedata, in int index, in int order, in float t, out vec4 p)
{
  int deg = order - 1;
  float u = 1.0 - t;

  float bc = 1.0;
  float tn = 1.0;
  vec4 result = texelFetch(curvedata, index);
  result = result * u;

  if (order > 2) {
    int i;
    for (i = 1; i <= deg - 1; ++i) {
      tn *= t;
      bc *= (float(deg-i+1) / float(i));
      result = (result + tn * bc * texelFetch(curvedata, index + i)) * u;
    }
    result += tn * t * texelFetch(curvedata, index + deg);
  } else {
    /* linear piece*/
    result = mix(texelFetch(curvedata, index), texelFetch(curvedata, index + 1), t);
  }

  /* project into euclidian coordinates */
  p = result;
  p[0] = p[0]/p[2];
  p[1] = p[1]/p[2];
}

/*****************************************************************************
 * binary search a LIST of sorted intervals :
 *
 *  buf       - samplerbuffer with sorted intervals
 *              buf[i][0] is minimum of i-th interval
 *              buf[i][1] is maximum of i-th interval
 *
 *  value     - value to look for (in which interval it is)
 *
 *  id        - index of first interval
 *
 *  intervals - number of intervals
 *
 *  result    - texture entry buf[i] in which the result is in
 *
 ****************************************************************************/
bool
binary_search ( in samplerBuffer buf,
                in float         value,
                in int           id,
                in int           intervals,
                inout vec4       result )
{
  result = vec4(0.0);

  int id_min = id;
  int id_max = id + intervals - 1;

  vec4 tmp = vec4(0.0);
  bool found = false;

  while (id_min <= id_max)
  {
    int id = id_min + (id_max - id_min) / int(2);
    tmp = texelFetch(buf, id);

    if (value >= tmp[0] && value <= tmp[1])
    {
      result = tmp;
      found    = true;
      break;
    } else {
      if (value < tmp[0])
      {
        id_max = id - 1;
      } else {
        id_min = id + 1;
      }
    }
  }

  if (found)
  {
    return found;
  } else {
    result = vec4(0.0);
    return found;
  }
}

/*****************************************************************************
 * optimized bisection method
 ****************************************************************************/
void
bisect(in samplerBuffer curvedata,
       in vec2 uv,
       in vec4 curve,
       in vec4 startend,
       inout int intersections)
{
	float t = 0.0;
  vec4 p  = vec4(0.0);

  vec4 p0 = vec4(startend[0], startend[1], 0.0, 0.0);
  vec4 p1 = vec4(startend[2], startend[3], 0.0, 0.0);

  const int CRITICAL_NUMERIC_ABORT = 32;
  int iters = 0;

  int index = int(curve[0]);
  int order = int(abs(curve[1]));
  bool horizontally_increasing = curve[1] < 0.0;

  float tmin = curve[2];
  float tmax = curve[3];

  //do
  //for (int i = 0; i < CRITICAL_NUMERIC_ABORT; ++i)
  int iterations = 0;

  while ( abs(uv[1]-p[1]) > TRIM_ERROR_TOLERANCE && CRITICAL_NUMERIC_ABORT > iterations)
  {
	  iterations++;

	  t = (tmax + tmin) / 2.0;
	  evaluateCurve(curvedata, index, order, t, p);

	  if (uv[1] > p[1]) {
	    tmin = (tmax + tmin) / 2.0;
	  } else {
	    tmax = (tmax + tmin) / 2.0;
	  }

	  if (!horizontally_increasing && uv[0] > p[0] && uv[1] > p[1])
    {
      break;
      //return 0;
      //return vec4(0.0, 1.0, 1.0, 1.0);
	  }

	  if (!horizontally_increasing && uv[0] < p[0] && uv[1] < p[1])
    {
      ++intersections;
      break;
      //return 1;
      //return vec4(1.0, 0.0, 1.0, 1.0);
	  }

	  if (horizontally_increasing && uv[0] < p[0] && uv[1] > p[1])
    {
      ++intersections;
      break;
      //return 1;
      //return vec4(1.0, 1.0, 0.0, 1.0);
	  }

	  if (horizontally_increasing && uv[0] > p[0] && uv[1] < p[1])
    {
      break;
      //return 0;
      //return vec4(1.0, 0.0, 0.5, 1.0);
	  }
	} //while ( abs(uv[1]-p[1]) > TRIM_ERROR_TOLERANCE && CRITICAL_NUMERIC_ABORT > iterations);
}


/*****************************************************************************
 * evaluate for parameter pair uv if it is trimmed or not
Calls to binary_search and bisect functions defined above
 ****************************************************************************/
void
trim(in samplerBuffer trimdata,
     in samplerBuffer celldata,
     in samplerBuffer curvelist,
     in samplerBuffer curvedata,
     in int trim_index,
     in bool trimtype,
     in vec2 uv)
{
  vec4 debug = vec4(0.2);
  vec4 domaininfo = texelFetch(trimdata, trim_index);

  if (int(domaininfo[2]) == 0)
  {
    return;
    //return vec4(1.0, 0.0, 1.0, 1.0);
  }

  int total_intersections = 0;

  // if there is no partition in vertical(v) direction -> return
  if (int(domaininfo[2]) == 0)
  {
    //return vec4(1.0);
    return;
  }

  vec4 vinterval = vec4(0.0, 0.0, 0.0, 0.0);
  bool vinterval_found = binary_search (trimdata, uv[1], trim_index + 1, int(domaininfo[2]), vinterval);

  if (!vinterval_found) {
    if (bool(trimtype)) {
      discard;  // outside of boundary and outer part is trimmed
    } else {
      return;   // outside boundary and inner part is trimmed
      //return vec4(0.0, 1.0, 1.0, 1.0);
    }
  }

  int celllist_id = int(vinterval[2]);
  int ncells      = int(vinterval[3]);

  vec4 celllist_info  = texelFetch(celldata, celllist_id);
  vec4 cell           = vec4(0.0);

  bool cellfound      = binary_search   (celldata, uv[0], celllist_id + 1, ncells, cell);
  if (!cellfound)
  {
    if (bool(trimtype)) {
      discard;  // outside of boundary and outer part is trimmed
    } else {
      return;   // outside boundary and inner part is trimmed
      //return vec4(0.0, 0.5, 0.5, 1.0);
    }
  }

  vec4 clist              = texelFetch(curvelist, int(cell[3]));
  total_intersections     = int(cell[2]);
  int curves_to_intersect = int(clist[0]);

  for (int i = 1; i <= curves_to_intersect; ++i)
  {
	  vec4 curveinfo1 = texelFetch(curvelist, int(cell[3] + 2*i - 1));
    vec4 curveinfo2 = texelFetch(curvelist, int(cell[3] + 2*i    ));

    bisect(curvedata, uv, curveinfo1, curveinfo2, total_intersections);
  }


  if (bool(trimtype))
  {
    ++total_intersections;
  }

  if ( mod(total_intersections, 2) == 1 )
  {
    discard;
  }


}

#endif

