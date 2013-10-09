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
flat in vec4 	vNormal[];

flat out vec3 	tcPosition[];
flat out int 	tcIndex[];
flat out vec2 	tcTessCoord[];

#define ID gl_InvocationID

vec4 to_screen_space(in vec3 point);

vec4 control_polygon_length(in samplerBuffer	data,
			    in int		offset,
			    in int 		u,
			    in int		v);

void
HornerBernstein(in samplerBuffer 	data,
		in int			offset,
		in int 			OrderU,
		in int 			OrderV,
		in vec2 		uv,
		out vec4 		du,
		out vec4		dv,
		out vec4 		point);

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
	return clamp(ceil(length / 8.0), 2.0, 64.0);
}

float inner_tesslevel()
{
	vec4 bbox_min = texelFetch(attribute_texture, int(vIndex[ID]) * 5 + 2);
	vec4 bbox_max = texelFetch(attribute_texture, int(vIndex[ID]) * 5 + 3);
	vec3 combin[8];
	int i, j;
	float max_length = 0.0;

	combin[0] = to_screen_space(vec3(bbox_min.x, bbox_min.y, bbox_min.z)).xyz;
	combin[1] = to_screen_space(vec3(bbox_min.x, bbox_min.y, bbox_max.z)).xyz;
	combin[2] = to_screen_space(vec3(bbox_min.x, bbox_max.y, bbox_min.z)).xyz;
	combin[3] = to_screen_space(vec3(bbox_min.x, bbox_max.y, bbox_max.z)).xyz;
	combin[4] = to_screen_space(vec3(bbox_max.x, bbox_min.y, bbox_min.z)).xyz;
	combin[5] = to_screen_space(vec3(bbox_max.x, bbox_min.y, bbox_max.z)).xyz;
	combin[6] = to_screen_space(vec3(bbox_max.x, bbox_max.y, bbox_min.z)).xyz;
	combin[7] = to_screen_space(vec3(bbox_max.x, bbox_max.y, bbox_max.z)).xyz;

	for ( i = 0; i < 7; i++ ) {
		for ( j = i + 1; j < 8; j++ ) {
			float temp_length = length(combin[i].xy - combin[j].xy);

			if ( temp_length > max_length ) {
				max_length = temp_length;
			}
		}
	}

	max_length = clamp(max_length, 0, 900);

	return clamp(ceil(max_length / 8.0), 2.0, 64.0);
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

vec4 to_screen_space(in vec3 point)
{
	vec4 ret_point;

	ret_point = projection_matrix * view_matrix * model_matrix * vec4(point, 1.0);
	ret_point = ret_point / ret_point.w;
	ret_point = vec4((ret_point.xy * 0.5 + 0.5) * 900.0, 1.0, 1.0);

	return ret_point;
}

void main()
{
	tcPosition[ID] 	= vPosition[ID];
	tcIndex[ID] 	= vIndex[ID];
	tcTessCoord[ID] = vTessCoord[ID];

	vec4 data = texelFetch(attribute_texture, int(vIndex[ID]) * 5);

	if ( frustum_cull() ) {
		if ( abs(vTessCoord[0].x - vTessCoord[1].x) * abs(vTessCoord[1].y - vTessCoord[2].y) == 1.0 ) {
			vec4 curve_factor = clamp(texelFetch(attribute_texture, int(vIndex[ID]) * 5 + 4), 1, 4);
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

			//Following three must be same for Ist Pass
			gl_TessLevelInner[0] = inner_tesslevel();
			gl_TessLevelOuter[1] = edge01;
			gl_TessLevelOuter[3] = edge32;

			//Following three must be same for Ist Pass
			gl_TessLevelInner[1] = inner_tesslevel();
			gl_TessLevelOuter[0] = edge20;
			gl_TessLevelOuter[2] = edge13;
		} else {
			vec4 point_on_plane0 = to_screen_space(vPosition[0]);
			vec4 point_on_plane1 = to_screen_space(vPosition[1]);
			vec4 point_on_plane2 = to_screen_space(vPosition[2]);
			vec4 point_on_plane3 = to_screen_space(vPosition[3]);

			//Approach I-----> For Outer Tessellation Levels : Take ratio according to the original control polygon length.
			//		   For Inner Tessellation Levels : Evaluate the mid point of the patch and get the diagonal length.

			vec4 edgelen = control_polygon_length(parameter_texture, int(data.x), int(data.y), int(data.z));

			vec2 p1 = mix(vTessCoord[0].xy, vTessCoord[1].xy, 0.5);
			vec2 p2 = mix(vTessCoord[3].xy, vTessCoord[2].xy, 0.5);

			vec2 mid_uv = mix(p1, p2, 0.5);
			vec4 du, dv, _puv;

			HornerBernstein(parameter_texture, int(data.x), int(data.y), int(data.z), mid_uv, du, dv, _puv);

			_puv = to_screen_space(_puv.xyz);

			float length1 = length(point_on_plane0.xy - _puv.xy) + length(_puv.xy - point_on_plane3.xy);
			float length2 = length(point_on_plane2.xy - _puv.xy) + length(_puv.xy - point_on_plane1.xy);

			float diagonal_length = min(length1, length2);

			float edge01 = edge_tesslevel(mix(edgelen[0], edgelen[2], abs(vTessCoord[0].y - vTessCoord[2].y)));
			float edge32 = edge_tesslevel(mix(edgelen[0], edgelen[2], abs(vTessCoord[0].y - vTessCoord[2].y)));
			float edge13 = edge_tesslevel(mix(edgelen[1], edgelen[3], abs(vTessCoord[0].x - vTessCoord[1].x)));
			float edge20 = edge_tesslevel(mix(edgelen[1], edgelen[3], abs(vTessCoord[0].x - vTessCoord[1].x)));

			/*****/

			//Approach II-----> For Outer Tessellation Levels : Approximate the curvature length of the edge according to the angle between its normals.
			//		    For Inner Tessellation Levels : Approximate the curvature of the surface according to the all edge normals.

			//Normals projected in XY-Plane
/*			vec2 normal0 = normalize(transpose(inverse(mat3(view_matrix * model_matrix))) * vNormal[0].xyz).xy;
			vec2 normal1 = normalize(transpose(inverse(mat3(view_matrix * model_matrix))) * vNormal[1].xyz).xy;
			vec2 normal2 = normalize(transpose(inverse(mat3(view_matrix * model_matrix))) * vNormal[2].xyz).xy;
			vec2 normal3 = normalize(transpose(inverse(mat3(view_matrix * model_matrix))) * vNormal[3].xyz).xy;

			float edge01 = edge_tesslevel(length(point_on_plane0 - point_on_plane1) * acos(dot(normal0, normal1)) / (2.0 * sin(acos(dot(normal0, normal1) / 2.0))));
			float edge32 = edge_tesslevel(length(point_on_plane3 - point_on_plane2) * acos(dot(normal3, normal2)) / (2.0 * sin(acos(dot(normal3, normal2) / 2.0))));
			float edge13 = edge_tesslevel(length(point_on_plane1 - point_on_plane3) * acos(dot(normal1, normal3)) / (2.0 * sin(acos(dot(normal1, normal3) / 2.0))));
			float edge20 = edge_tesslevel(length(point_on_plane2 - point_on_plane0) * acos(dot(normal2, normal0)) / (2.0 * sin(acos(dot(normal2, normal0) / 2.0))));*/

			//Following three must be same for Ist Pass
			gl_TessLevelInner[0] = edge_tesslevel(diagonal_length);
			gl_TessLevelOuter[1] = edge01;
			gl_TessLevelOuter[3] = edge32;

			//Following three must be same for Ist Pass
			gl_TessLevelInner[1] = edge_tesslevel(diagonal_length);
			gl_TessLevelOuter[0] = edge20;
			gl_TessLevelOuter[2] = edge13;
		}
	} else {
		gl_TessLevelInner[0] = 1;
		gl_TessLevelOuter[1] = 1;
		gl_TessLevelOuter[3] = 1;

		gl_TessLevelInner[1] = 1;
		gl_TessLevelOuter[0] = 1;
		gl_TessLevelOuter[2] = 1;
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

void
HornerBernstein(in samplerBuffer 	data,
		in int			offset,
		in int 			OrderU,
		in int 			OrderV,
		in vec2 		uv,
		out vec4 		du,
		out vec4		dv,
		out vec4 		point)
{
	int i, j;
	int n = OrderU - 1;
	int m = OrderV - 1;
	float powu, powv, nci, mcj;

	vec4 tpoint0;
	vec4 tpointi;
	vec4 tpointn; 			//For i == n
	point = vec4(0.0);

	//For First and Last Rows
	tpoint0 = tpointn = vec4(0.0);
	nci = mcj = powu = powv = 1.0;

	for ( j = 0; j < m; j++ ) {
		tpoint0 = (tpoint0 + mcj * powv * texelFetch(data, offset + OrderU * j)) * (1.0 - uv[1]);
		tpointn = (tpointn + mcj * powv * texelFetch(data, offset + OrderU * j + n)) * (1.0 - uv[1]);
		mcj *= float(m - j) / float(j + 1);
		powv *= uv[1];
	}

	tpoint0 = tpoint0 + powv * texelFetch(data, offset + OrderU * j);
	tpointn = tpointn + powv * texelFetch(data, offset + OrderU * j + n);
	point = (point + nci * powu * tpoint0) * (1.0 - uv[0]);
	nci = nci * float(n);
	powu = powu * uv[0];

	//For Remaining Rows
	for ( i = 1; i < n; i++ ) {
		tpointi = vec4(0.0);
		mcj = 1.0;
		powv = 1.0;

		for ( j = 0; j < m; j++ ) {
			tpointi = (tpointi + mcj * powv * texelFetch(data, offset + OrderU * j + i)) * (1.0 - uv[1]);
			mcj *= float(m - j) / float(j + 1);
			powv *= uv[1];
		}

		tpointi = tpointi + powv * texelFetch(data, offset + OrderU * j + i);
		point = (point + nci * powu * tpointi) * (1.0 - uv[0]);
		nci = nci * float(n - i) / float(i + 1);
		powu = powu * uv[0];
	}

	point = point + powu * tpointn;

	//Partial Derivative with respect to u   dP/du
	powu = powv = nci = mcj = 1.0;
	du = tpoint0 = tpointn = vec4(0.0);

	for ( j = 0; j < m; j++ ) {
		tpoint0 = (tpoint0 + mcj * powv * (texelFetch(data, offset + OrderU * j + 1) - texelFetch(data, offset + OrderU * j) ) ) * (1.0 - uv[1]);
		tpointn = (tpointn + mcj * powv * (texelFetch(data, offset + OrderU * j + n) - texelFetch(data, offset + OrderU * j + n - 1)) ) * (1.0 - uv[1]);
		mcj *= float(m - j) / float(j + 1);
		powv *= uv[1];
	}

	tpoint0 = tpoint0 + powv * (texelFetch(data, offset + OrderU * j + 1) - texelFetch(data, offset + OrderU * j));
	tpointn = tpointn + powv * (texelFetch(data, offset + OrderU * j + n) - texelFetch(data, offset + OrderU * j + n - 1));
	du = (du + nci * powu * tpoint0) * (1.0 - uv[0]);
	nci = nci * float(n - 1);
	powu = powu * uv[0];

	//For Remaining Rows
	for ( i = 1; i < n - 1; i++ ) {
		tpointi = vec4(0.0);
		mcj = 1.0;
		powv = 1.0;

		for ( j = 0; j < m; j++ ) {
			tpointi = (tpointi + mcj * powv * (texelFetch(data, offset + OrderU * j + i + 1) - texelFetch(data, offset + OrderU * j + i))) * (1.0 - uv[1]);
			mcj *= float(m - j) / float(j + 1);
			powv *= uv[1];
		}

		tpointi = tpointi + powv * (texelFetch(data, offset + OrderU * j + i + 1) - texelFetch(data, offset + OrderU * j + i));
		du = (du + nci * powu * tpointi) * (1.0 - uv[0]);
		nci = nci * float(n - i - 1) / float(i + 1);
		powu = powu * uv[0];
	}

	du = du + powu * tpointn;
	du = du * n;

	//Partial Derivative with respect to v   dP/dv
	dv = vec4(0.0);
	powu = 1.0;
	powv = 1.0;
	nci = 1.0; mcj = 1.0;
	tpoint0 = vec4(0.0);
	tpointn = vec4(0.0);

	for ( j = 0; j < m - 1; j++ ) {
		tpoint0 = (tpoint0 + mcj * powv * ( texelFetch(data, offset + OrderU * (j + 1)) - texelFetch(data, offset + OrderU * j) ) ) * (1.0 - uv[1]);
		tpointn = (tpointn + mcj * powv * (texelFetch(data, offset + OrderU * (j + 1) + n) - texelFetch(data, offset + OrderU * j + n)) ) * (1.0 - uv[1]);
		mcj *= float(m - j - 1) / float(j + 1);
		powv *= uv[1];
	}

	tpoint0 = tpoint0 + powv * (texelFetch(data, offset + OrderU * (j + 1)) - texelFetch(data, offset + OrderU * j));
	tpointn = tpointn + powv * (texelFetch(data, offset + OrderU * (j + 1) + n) - texelFetch(data, offset + OrderU * j + n));

	dv = (dv + nci * powu * tpoint0) * (1.0 - uv[0]);
	nci = nci * float(n);
	powu = powu * uv[0];

	//For Remaining Rows
	for ( i = 1; i < n; i++ ) {
		tpointi = vec4(0.0);
		mcj = 1.0;
		powv = 1.0;

		for ( j = 0; j < m - 1; j++ ) {
			tpointi = (tpointi + mcj * powv * (texelFetch(data, offset + OrderU * (j + 1) + i) - texelFetch(data, offset + OrderU * j + i))) * (1.0 - uv[1]);
			mcj *= float(m - j - 1) / float(j + 1);
			powv *= uv[1];
		}

		tpointi = tpointi + powv * (texelFetch(data, offset + OrderU * (j + 1) + i) - texelFetch(data, offset + OrderU * j + i));
		dv = (dv + nci * powu * tpointi) * (1.0 - uv[0]);
		nci = nci * float(n - i) / float(i + 1);
		powu = powu * uv[0];
	}

	dv = dv + powu * tpointn;
	dv = dv * m;

	//Transformation from Homogeneous Space to Euclidean Space
	du = (du * point.w - point * du.w) / pow(point.w, 2);		//P.S.:  simply p(t) = r(t) / s(t) => p'(t) = (r'(t)s(t)-r(t)s'(t)) / (s(t)^2) .....
        dv = (dv * point.w - point * dv.w) / pow(point.w, 2);		//P.S.: p(t) = r(t) / s(t) => r(t) = p(t) x s(t) => r'(t) = p'(t)xs(t) + p(t)xs'(t) => p'(t) = ..... doesn't give precise results
        point = point / point.w;
}

#endif
