#if 0

#version 400 core

layout(triangles) in;
layout(triangle_strip, max_vertices = 3) out;

uniform mat4 projection_matrix;
uniform mat4 model_matrix;
uniform mat4 view_matrix;
uniform mat4 normal_matrix;

flat in vec3 	tePosition[3];
flat in int 	teIndex[3];
flat in vec2 	teTessCoord[3];
flat in vec4 	teNormal[3];
flat in vec2 	teTemp[3];

out vec3 	gPosition;
flat out float 	gIndex;
out vec2 	gTessCoord;
out vec4 	gNormal;
flat out vec2 	gTemp;

uniform samplerBuffer parameter_texture;
uniform samplerBuffer attribute_texture;

void evaluateSurface(in samplerBuffer data, in int index, in int orderU, in int orderV, in vec2 uv, out vec4 p, out vec4 du, out vec4 dv);
void HornerBernstein(in samplerBuffer data, in int offset, in int OrderU, in int OrderV, in vec2 uv, out vec4 du, out vec4 dv, out vec4 point);

void main()
{
    //First Emitted Vertex
    gPosition 	= (view_matrix * model_matrix * vec4(tePosition[0], 1.0)).xyz;
    gIndex 	= float(teIndex[0]);
    gTessCoord 	= teTessCoord[0];
    gNormal 	= vec4(normalize(mat3(transpose(inverse(view_matrix * model_matrix))) * teNormal[0].xyz), 1.0 );
    gTemp 	= teTemp[0];
    gl_Position = projection_matrix * view_matrix * model_matrix * gl_in[0].gl_Position; EmitVertex();

    //Second Emitted Vertex
    gPosition 	= (view_matrix * model_matrix * vec4(tePosition[1], 1.0)).xyz;
    gIndex 	= float(teIndex[1]);
    gTessCoord 	= teTessCoord[1];
    gNormal 	= vec4(normalize(mat3(transpose(inverse(view_matrix * model_matrix))) * teNormal[1].xyz), 1.0 );
    gTemp 	= teTemp[1];
    gl_Position = projection_matrix * view_matrix * model_matrix * gl_in[1].gl_Position; EmitVertex();

    //Third Emitted Vertex
    gPosition 	= (view_matrix * model_matrix * vec4(tePosition[2], 1.0)).xyz;
    gIndex 	= float(teIndex[2]);
    gTessCoord 	= teTessCoord[2];
    gNormal 	= vec4(normalize(mat3(transpose(inverse(view_matrix * model_matrix))) * teNormal[2].xyz), 1.0 );
    gTemp 	= teTemp[2];
    gl_Position = projection_matrix * view_matrix * model_matrix * gl_in[2].gl_Position; EmitVertex();

    EndPrimitive();
}

void
evaluateSurface(in samplerBuffer  data,
                in int            index,
                in int            orderU,
                in int            orderV,
                in vec2           uv,
                out vec4          p,
                out vec4          du,
                out vec4          dv)
{
  p  = vec4(0.0);
  du = vec4(0.0);
  dv = vec4(0.0);

  float bcu = 1.0;
  float un  = 1.0;

  int deg_u = orderU - 1;
  int deg_v = orderV - 1;

  vec4 u0_0 = texelFetch(data, index              ) * (1.0 - uv[0]);
  vec4 u0_1 = texelFetch(data, index + 1          ) * (1.0 - uv[0]);
  vec4 u1_0 = texelFetch(data, index + orderU    )  * (1.0 - uv[0]);
  vec4 u1_1 = texelFetch(data, index + orderU + 1)  * (1.0 - uv[0]);

  /**************************************** 1. step : horner for first 2 rows *********************************/
  int i;
  for (i = 1; i <= deg_u - 2; ++i) {
    un = un * uv[0];
    bcu = bcu * (float(deg_u - i) / float(i));

    u0_0 = (u0_0 + un * bcu * texelFetch(data, index + i    ))           * (1.0 - uv[0]);
    u0_1 = (u0_1 + un * bcu * texelFetch(data, index + i + 1))           * (1.0 - uv[0]);

    u1_0 = (u1_0 + un * bcu * texelFetch(data, index + orderU + i    )) * (1.0 - uv[0]);
    u1_1 = (u1_1 + un * bcu * texelFetch(data, index + orderU + i + 1)) * (1.0 - uv[0]);
  }

  u0_0 += un * uv[0] * texelFetch(data, index          + deg_u - 1);
  u0_1 += un * uv[0] * texelFetch(data, index          + deg_u    );
  u1_0 += un * uv[0] * texelFetch(data, index + orderU + deg_u - 1);
  u1_1 += un * uv[0] * texelFetch(data, index + orderU + deg_u    );

  /* point in first and second row */
  vec4 u0 = (1.0 - uv[0]) * u0_0 + uv[0] * u0_1;
  vec4 u1 = (1.0 - uv[0]) * u1_0 + uv[0] * u1_1;

  /**************************************** 2. step : inner loop for rows 3 to order - 1 ***********************/
  float bcv = 1.0;
  float vn = 1.0;

  vec4 v0 = u0 * (1.0 - uv[1]);
  vec4 v1 = u1 * (1.0 - uv[1]);

  vec4 ui, ui_0, ui_1;
  for (i = 1; i <= deg_v - 2; ++i)
  {
    bcu = 1.0;
    un = 1.0;
    ui_0 = texelFetch(data, index + (i+1) * orderU)     * (1.0 - uv[0]);
    ui_1 = texelFetch(data, index + (i+1) * orderU + 1) * (1.0 - uv[0]);

    int j;
    for (j = 1; j <= deg_u-2; ++j)
    {
      un = un * uv[0];
      bcu = bcu * (float(deg_u - j) / float(j));
      ui_0 = (ui_0 + un * bcu * texelFetch(data, index + (i+1) * orderU + j    )) * (1.0 - uv[0]);
      ui_1 = (ui_1 + un * bcu * texelFetch(data, index + (i+1) * orderU + j + 1)) * (1.0 - uv[0]);
    }
    ui_0 = ui_0 + un * uv[0] * texelFetch(data, index + (i+1) * orderU + deg_u - 1);
    ui_1 = ui_1 + un * uv[0] * texelFetch(data, index + (i+1) * orderU + deg_u    );
    ui = (1.0 - uv[0]) * ui_0 + uv[0] * ui_1;

    u0 = u1;
    u1 = ui;

    vn = vn * uv[1];
    bcv = bcv * (float(deg_v - i) / float(i));
    v0 = (v0 + vn * bcv * u0) * (1.0 - uv[1]);
    v1 = (v1 + vn * bcv * u1) * (1.0 - uv[1]);
  }

  /**************************************** 3. step : horner scheme for last row *******************************/
  bcu = 1.0;
  un = 1.0;
  ui_0 = texelFetch(data, index + deg_v * orderU    ) * (1.0 - uv[0]);
  ui_1 = texelFetch(data, index + deg_v * orderU + 1) * (1.0 - uv[0]);

  for (i = 1; i <= deg_u-2; ++i)
  {
    un = un * uv[0];
    bcu = bcu * (float(deg_u-i) / float(i));
    ui_0 = (ui_0 + un * bcu * texelFetch(data, index + deg_v * orderU + i    )) * (1.0 - uv[0]);
    ui_1 = (ui_1 + un * bcu * texelFetch(data, index + deg_v * orderU + i + 1)) * (1.0 - uv[0]);
  }

  ui_0 = ui_0 + un * uv[0] * texelFetch(data, index + deg_v * orderU + deg_u - 1);
  ui_1 = ui_1 + un * uv[0] * texelFetch(data, index + deg_v * orderU + deg_u    );
  ui = (1.0 - uv[0]) * ui_0 + uv[0] * ui_1;

  /**************************************** 4. step : final interpolation over v ********************************/
  v0 += vn * uv[1] * u1;
  v1 += vn * uv[1] * ui;

  p = (1.0 - uv[1]) * v0 + uv[1] * v1;

  /* transform to euclidian space */
  dv = (orderV - 1) * ((v0[3] * v1[3]) / (p[3] * p[3])) * ((v1 / v1[3]) - (v0 / v0[3]));

  /************************************ 5.step : dartial derivative over u ***********************************/
  bcv = 1.0;
  vn = 1.0;

  vec4 v0_0 = texelFetch(data, index              ) * (1.0 - uv[1]);
  vec4 v0_1 = texelFetch(data, index + orderU    ) * (1.0 - uv[1]);
  vec4 v1_0 = texelFetch(data, index + 1          ) * (1.0 - uv[1]);
  vec4 v1_1 = texelFetch(data, index + orderU + 1) * (1.0 - uv[1]);

  for (i = 1; i <= deg_v - 2; ++i)
  {
    vn = vn * uv[1];
    bcv = bcv * (float(deg_v - i) / float(i));

    v0_0 = (v0_0 + vn * bcv * texelFetch(data, index + (i  ) * orderU    )) * (1.0 - uv[1]);
    v0_1 = (v0_1 + vn * bcv * texelFetch(data, index + (i+1) * orderU    )) * (1.0 - uv[1]);

    v1_0 = (v1_0 + vn * bcv * texelFetch(data, index + (i  ) * orderU + 1)) * (1.0 - uv[1]);
    v1_1 = (v1_1 + vn * bcv * texelFetch(data, index + (i+1) * orderU + 1)) * (1.0 - uv[1]);
  }

  v0_0 = v0_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU    );
  v0_1 = v0_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU    );
  v1_0 = v1_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU + 1);
  v1_1 = v1_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU + 1);

  /* point in first and second row */
  v0 = (1.0 - uv[1]) * v0_0 + uv[1] * v0_1;
  v1 = (1.0 - uv[1]) * v1_0 + uv[1] * v1_1;

  /*********************************** 6. step : for all columns *******************************************/
  bcu = 1.0;
  un = 1.0;

  u0 = v0 * (1.0 - uv[0]);
  u1 = v1 * (1.0 - uv[0]);

  vec4 vi_0, vi_1, vi;
  for (i = 1; i <= deg_u - 2; ++i)
  {
    bcv = 1.0;
    vn = 1.0;
    vi_0 = texelFetch(data, index +           i + 1) * (1.0 - uv[1]);
    vi_1 = texelFetch(data, index + orderU + i + 1) * (1.0 - uv[1]);

    int j;
    for (j = 1; j <= deg_v-2; ++j)
    {
      vn = vn * uv[1];
      bcv = bcv * (float(deg_v - j) / float(j));
      vi_0 = (vi_0 + vn * bcv * texelFetch(data, index + (j  ) * orderU + i + 1)) * (1.0 - uv[1]);
      vi_1 = (vi_1 + vn * bcv * texelFetch(data, index + (j+1) * orderU + i + 1)) * (1.0 - uv[1]);
    }
    vi_0 = vi_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU + i + 1);
    vi_1 = vi_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU + i + 1);
    vi = (1.0 - uv[1]) * vi_0 + uv[1] * vi_1;

    v0 = v1;
    v1 = vi;

    un = un * uv[0];
    bcu = bcu * (float(deg_u - i) / float(i));
    u0 = (u0 + un * bcu * v0) * (1.0 - uv[0]);
    u1 = (u1 + un * bcu * v1) * (1.0 - uv[0]);
  }

  /********************************* 7. horner step for last column ****************************************/
  bcv = 1.0;
  vn = 1.0;
  vi_0 = texelFetch(data, index +           deg_u      ) * (1.0 - uv[1]);
  vi_1 = texelFetch(data, index + orderU + deg_u      ) * (1.0 - uv[1]);

  for (i = 1; i <= deg_v-2; ++i)
  {
    vn = vn * uv[1];
    bcv = bcv * (float(deg_v-i) / float(i));
    vi_0 = (vi_0 + vn * bcv * texelFetch(data, index + (i  ) * orderU + deg_u)) * (1.0 - uv[1]);
    vi_1 = (vi_1 + vn * bcv * texelFetch(data, index + (i+1) * orderU + deg_u)) * (1.0 - uv[1]);
   }

  vi_0 = vi_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU + deg_u);
  vi_1 = vi_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU + deg_u);
  vi = (1.0 - uv[1]) * vi_0 + uv[1] * vi_1;

  /******************************* 8. final interpolation ***************************************************/
  u0 += un * uv[0] * v1;
  u1 += un * uv[0] * vi;

  /* transform to euclidian space */
  du = deg_u * ((u0[3] * u1[3]) / (p[3] * p[3])) * ((u1 / u1[3]) - (u0 / u0[3]));
  p = p/p[3];
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
	du = (du - point * du.w) / point.w;		//P.S.: p(t) = r(t) / s(t) => r(t) = p(t) x s(t) => r'(t) = p'(t)xs(t) + p(t)xs'(t) => p'(t) = .....
        dv = (dv - point * dv.w) / point.w;		//      or simply p(t) = r(t) / s(t) => p'(t) = (r'(t)s(t)-r(t)s'(t)) / (s(t)^2) .....
        point = point / point.w;
}

#endif

