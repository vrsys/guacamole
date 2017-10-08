@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "common/gua_camera_uniforms.glsl"

layout (lines_adjacency) in;
layout (triangle_strip, max_vertices = 14) out;

in VertexData {
  vec3  gua_varying_object_position;
  vec4  gua_varying_color_rgba;
  float gua_varying_thickness;
  vec3  gua_varying_rme;
} VertexIn[4];

out VertexData {
  vec3 gua_varying_world_position;
  vec3 gua_varying_normal;
  vec4 gua_varying_color_rgba;
  vec3 gua_varying_rme;
  vec3 gua_start_point_ws_pos;
  vec3 gua_end_point_ws_pos;
} VertexOut;

const int ordered_line_strip_indices[14] = {3, 2, 6, 7, 4, 2, 0, 3, 1, 6,   5, 4, 1, 0};

vec4 prismoid[8];

void main() {

  vec3 cube_object_space_center = VertexIn[0].gua_varying_object_position;
  float cube_half_side_length = VertexIn[0].gua_varying_thickness;

  vec3 p0, p1, p2, p3;
  p0 = VertexIn[0].gua_varying_object_position; p1 = VertexIn[1].gua_varying_object_position;
  p2 = VertexIn[2].gua_varying_object_position; p3 = VertexIn[3].gua_varying_object_position;
  
  vec3 n0 = normalize(p1-p0);
  vec3 n1 = normalize(p2-p1);
  vec3 n2 = normalize(p3-p2);
  vec3 u = normalize(n0+n1);
  vec3 v = normalize(n1+n2);

  vec3 i;
  vec3 j;

  vec3 k;


  vec3 normal_1_2[2];

  float r = cube_half_side_length * 2;

    // Compute face 1 of 2:
    j = u; 
    //i = vec3(0.0, 1.0, 0.0);


    //vec3 half_way_through_u = p0 + 0.5*u;
    //vec3 hw_t_u_to_p1 = normalize(p1 - half_way_through_u);

    vec3 mirrored_p1_along_u = -n0 + n1;
    vec3 p1_to_mirrored_p1 = normalize(mirrored_p1_along_u - p1);
    //vec3 sounding_line_point_on_u = p0 + (dot(n0, u) / dot(u, u)) * u;//  0.5*v;
    //vec3 hw_t_u_to_p1 = normalize(p1 - sounding_line_point_on_u);

    i = normalize(cross(p1_to_mirrored_p1, u));

/*
    if( dot(i, vec3(0.0f, 99999.9f, 0.0f )) < 0.0) {
      i *= -1;
    } 
*/
    normal_1_2[0] = i;
    //i = normalize(cross(n0, -n1));
    //i = normalize(n0 - n1);
    k = cross(i, j); 
    i *= r;
    k *= r;
    /*
    prismoid[0] = vec4(p1 + i + k, 1.0);
    prismoid[1] = vec4(p1 + i - k, 1.0);
    prismoid[2] = vec4(p1 - i - k, 1.0);
    prismoid[3] = vec4(p1 - i + k, 1.0);
    */
    prismoid[0] = vec4(p1 + i + k, 1.0);
    prismoid[1] = vec4(p1 + i - k, 1.0);
    prismoid[2] = vec4(p1 - i - k, 1.0);
    prismoid[3] = vec4(p1 - i + k, 1.0);
    


    // Compute face 2 of 2:
    j = v; 


    vec3 mirrored_p2_along_v = -n1 + n2;
    vec3 p2_to_mirrored_p2 = normalize(mirrored_p2_along_v - p2);

    //vec3 sounding_line_point_on_v = p1 + (dot(n1, v) / dot(v, v)) * v;//  0.5*v;
    //vec3 hw_t_v_to_p2 = normalize(p2 - sounding_line_point_on_v);

    i = normalize(cross(p2_to_mirrored_p2, v));

/*
    if( dot(i, vec3(0.0f, 0.0f, 0.0f )) < 0.0) {
      i *= -1;
    } 
*/

    normal_1_2[1] = i;
    //i = vec3(0.0, 1.0, 0.0); 
    //i = normalize(n1 - n2);
    //i = normalize(cross(n1, -n2));
    k = cross(i, j); i *= r; k *= r;
    prismoid[4] = vec4(p2 + i + k, 1.0);
    prismoid[5] = vec4(p2 + i - k, 1.0);
    prismoid[6] = vec4(p2 - i - k, 1.0);
    prismoid[7] = vec4(p2 - i + k, 1.0);

  /* = {
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, 1.0, -1.0) ), 1.0), //vertex 0
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, 1.0, -1.0) ), 1.0), //vertex 1
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0,  1.0,  1.0) ), 1.0), //vertex 2
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0,  1.0, 1.0) ), 1.0), //vertex 3
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, -1.0, -1.0) ), 1.0), //vertex 4
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, -1.0, -1.0) ), 1.0), //vertex 5
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, -1.0,  1.0) ), 1.0), //vertex 6
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, -1.0,  1.0) ), 1.0), //vertex 7
  };*/

  VertexOut.gua_varying_normal         = vec3(1.0, 0.0, 0.0);
  VertexOut.gua_varying_color_rgba     = VertexIn[0].gua_varying_color_rgba;
  VertexOut.gua_varying_rme            = VertexIn[0].gua_varying_rme;


  mat4 gua_view_projection_matrix = gua_projection_matrix * gua_view_matrix;
  for(int v_idx = 0; v_idx < 14; ++v_idx) {

    uint prismoid_vertex_index = ordered_line_strip_indices[v_idx];

    if(prismoid_vertex_index< 3) {
        VertexOut.gua_varying_color_rgba = VertexIn[1].gua_varying_color_rgba;
        VertexOut.gua_varying_normal = (gua_normal_matrix * vec4(normal_1_2[0], 0.0)).xyz ;
    } else {
        VertexOut.gua_varying_color_rgba = VertexIn[2].gua_varying_color_rgba;
        VertexOut.gua_varying_normal = (gua_normal_matrix * vec4(normal_1_2[1], 0.0)).xyz ;
    }

    vec4 model_transformed_vertex = gua_model_matrix * prismoid[prismoid_vertex_index];
    VertexOut.gua_varying_world_position = model_transformed_vertex.xyz;
    gl_Position = gua_view_projection_matrix * model_transformed_vertex;

    vec3 gua_start_point_ws_pos = (gua_model_matrix * vec4(VertexIn[1].gua_varying_object_position, 1.0) ).xyz;
    vec3 gua_end_point_ws_pos = (gua_model_matrix * vec4(VertexIn[1].gua_varying_object_position, 1.0) ).xyz;

    EmitVertex();
  }

  EndPrimitive();



/*
  if(enable_backface_culling == false || VertexIn[0].pass_normal.z > 0.0) {

      // --------------------------- common attributes -----------------------------------
      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].pass_ms_u,
                              VertexIn[0].pass_ms_v);

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        VertexOut.pass_uv_coords        = uv_multiplier.yz;
        vec4 q_pos_ms                   = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position                     = gua_model_view_projection_matrix * q_pos_ms;
        VertexOut.pass_world_position   = (gua_model_matrix * q_pos_ms).xyz;
        EmitVertex();
      }

      EndPrimitive();
  }
*/
}



