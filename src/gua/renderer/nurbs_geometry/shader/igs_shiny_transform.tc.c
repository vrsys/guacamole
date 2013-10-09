#if 0

#version 400 core

layout(vertices = 4) out;

uniform samplerBuffer parameter_texture;
uniform samplerBuffer attribute_texture;

uniform mat4 projection_matrix;
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 normal_matrix;

flat in vec3 	vPosition[];
flat in int 	vIndex[];
flat in vec2 	vTessCoord[];

flat out vec3 	tcPosition[];
flat out int 	tcIndex[];
flat out vec2 	tcTessCoord[];

#define ID gl_InvocationID

vec4 control_polygon_length(in samplerBuffer	data,
			    in int		offset,
			    in int 		u,
			    in int		v);

float edge_length(in vec3 v1, in vec3 v2)
{
	vec4 cs_v1 = projection_matrix * view_matrix * model_matrix * vec4(v1, 1.0);
	vec4 cs_v2 = projection_matrix * view_matrix * model_matrix * vec4(v2, 1.0);

	cs_v1 = cs_v1 / cs_v1.w;
	cs_v2 = cs_v2 / cs_v2.w;

	cs_v1 = (cs_v1 * 0.5 + 0.5) * 900;
	cs_v2 = (cs_v2 * 0.5 + 0.5) * 900;

	vec2 edge = cs_v1.xy - cs_v2.xy;

	return clamp(length(edge), 0, 900);
}

float edge_tesslevel(in float length)
{
	return clamp(length / (64.0 * 8.0), 1, 64);
}

bool isInside(vec4 point)
{
	if( (point.x >= -1.0 && point.x <= 1.0) && (point.y >= -1.0 && point.y <= 1.0) && (point.z >= -1.0 && point.z <= 1.0) ) {
		return true;
	}

	return false;
}

bool frustum_cull()
{
	//Frustum Culling in Clip Space

	vec4 cs_v1 = projection_matrix * view_matrix * model_matrix * vec4(vPosition[0], 1.0);
	vec4 cs_v2 = projection_matrix * view_matrix * model_matrix * vec4(vPosition[1], 1.0);
	vec4 cs_v3 = projection_matrix * view_matrix * model_matrix * vec4(vPosition[2], 1.0);
	vec4 cs_v4 = projection_matrix * view_matrix * model_matrix * vec4(vPosition[3], 1.0);

	cs_v1 = cs_v1 / cs_v1.w;
	cs_v2 = cs_v2 / cs_v2.w;
	cs_v3 = cs_v3 / cs_v3.w;
	cs_v4 = cs_v4 / cs_v4.w;

	if ( isInside(cs_v1) || isInside(cs_v2) || isInside(cs_v3) || isInside(cs_v4) ) {
		return true;
	}

	return false;
}

void main()
{
	tcPosition[ID] 	= vPosition[ID];
	tcIndex[ID] 	= vIndex[ID];
	tcTessCoord[ID] = vTessCoord[ID];

	if (frustum_cull()) {
		vec4 data = texelFetch(attribute_texture, int(vIndex[ID]) * 5);
		vec4 edgelen = control_polygon_length(parameter_texture, int(data.x), int(data.y), int(data.z));

		//	    2
		//	2------3
		//3	|      |      1
		//      |      |
		//      0------1
		//	    0

		float edge01 = edge_tesslevel(edgelen[0]);
		float edge32 = edge_tesslevel(edgelen[2]);
		float edge13 = edge_tesslevel(edgelen[1]);
		float edge20 = edge_tesslevel(edgelen[3]);

		float tess_level = max(max(edge01, edge20), max(edge32, edge13));

		//Following three must be same for Ist Pass
		gl_TessLevelInner[0] = tess_level;
		gl_TessLevelOuter[1] = tess_level;
		gl_TessLevelOuter[3] = tess_level;

		//Following three must be same for Ist Pass
		gl_TessLevelInner[1] = tess_level;
		gl_TessLevelOuter[0] = tess_level;
		gl_TessLevelOuter[2] = tess_level;
	} else {
		gl_TessLevelInner[0] = 0;
		gl_TessLevelOuter[1] = 0;
		gl_TessLevelOuter[3] = 0;

		gl_TessLevelInner[1] = 0;
		gl_TessLevelOuter[0] = 0;
		gl_TessLevelOuter[2] = 0;
	}
}

vec4 control_polygon_length(in samplerBuffer	data,
			    in int		offset,
			    in int 		u,
			    in int		v)
{
	int i, j;
	vec4 output = vec4(0.0);
	vec4 first, second;
	mat4 mvp = projection_matrix * view_matrix * model_matrix;

//	2------3
//	|      |
//      |      |
//      0------1

	/*For Edge 01*/
	for ( i = 1; i < u; ++i ) {
		output[0] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - 1).xyz);
	}

	/*For Edge 13*/
	for ( i = 2 * u - 1; i < u * v; i += u ) {
		output[1] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - u).xyz);
	}

	/*For Edge 23*/
	for ( i = u * v - u + 1; i < u * v; ++i ) {
		output[2] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - 1).xyz);
	}

	/*For Edge 02*/
	for ( i = u; i <= u * v - u; i += u ) {
		output[3] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - u).xyz);
	}
/**/
	return output;
}

#endif
