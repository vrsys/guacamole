@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/gua_camera_uniforms.glsl"

in VertexData {
  vec3 ms_pos;
  vec3 ms_u;
  vec3 ms_v;
  vec3 ms_normal;
} VertexIn[1];
//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mv_matrix;
uniform mat4 kinect_mvp_matrix;
uniform float point_size = 1.0;
uniform float voxel_half_size = 0.0;

const vec3 conservative_bb_limit_min = vec3(-1.5, -0.5, -1.5);
const vec3 conservative_bb_limit_max = vec3( 1.5,  2.5,  1.5);
const vec3 quant_steps               = vec3( (conservative_bb_limit_max.x - conservative_bb_limit_min.x) / (1<<14),
                                             (conservative_bb_limit_max.y - conservative_bb_limit_min.y) / (1<<13),
                                             (conservative_bb_limit_max.z - conservative_bb_limit_min.z) / (1<<13));

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////

const uvec4 shift_vector = uvec4(18, 5, 0, 24);
const uvec4 mask_vector  = uvec4(0x3FFFu, 0x1FFFu, 0x1F, 0xFFu);

layout (points) in;
layout (triangle_strip, max_vertices = 4) out;


const int ordered_line_strip_indices[14] = {3, 2, 6, 7, 4, 2, 0, 3, 1, 6,   5, 4, 1, 0};

out vec2 uv_coords;
//out float es_half_cube_side_length;
//out float eye_space_depth;
void main() {
/*
  vec3 cube_object_space_center = VertexIn[0].ms_pos;
  float cube_half_side_length = voxel_half_size * 2.0;

  vec4 cube_vertex_object_space_positions[8] = {
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, 1.0, -1.0) ), 1.0), //vertex 0
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, 1.0, -1.0) ), 1.0), //vertex 1
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0,  1.0,  1.0) ), 1.0), //vertex 2
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0,  1.0, 1.0) ), 1.0), //vertex 3
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, -1.0, -1.0) ), 1.0), //vertex 4
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, -1.0, -1.0) ), 1.0), //vertex 5
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3(-1.0, -1.0,  1.0) ), 1.0), //vertex 6
    vec4( vec3(cube_object_space_center + vec3(cube_half_side_length) * vec3( 1.0, -1.0,  1.0) ), 1.0), //vertex 7
  };


  //compute view space voxel diagonal length (for now in the shader, can later be done for all different sizes once and put into ubo)
  float view_space_voxel_diag_length = length(kinect_mv_matrix * vec4(cube_half_side_length, cube_half_side_length, cube_half_side_length, 0.0));

  //es_half_cube_side_length = view_space_voxel_diag_length;

  for(int v_idx = 0; v_idx < 4; ++v_idx) {
    vec4 model_view_transformed_vertex = kinect_mv_matrix * cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]];

    vec4 projected_vertex = gua_projection_matrix * model_view_transformed_vertex;


    //gl_Position = kinect_mvp_matrix * cube_vertex_object_space_positions[ordered_line_strip_indices[v_idx]];

    gl_Position = projected_vertex;

    gl_Position.z = (-(model_view_transformed_vertex.z - 4.5*view_space_voxel_diag_length)) / gua_clip_far;
    gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
    gl_Position.z  *= gl_Position.w;

    EmitVertex();
  }

  EndPrimitive();
*/


  //vec4 object_normal = vec4(normalize(VertexIn[0].ms_normal), 0.0);
  //vec4 view_normal  = inverse_transpose_model_view_matrix * object_normal;

  //if(enable_backface_culling == false || view_normal.z > 0.0 ) {

      // --------------------------- common attributes -----------------------------------

      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].ms_u,
                              VertexIn[0].ms_v);

      float es_linear_depth_center = (kinect_mv_matrix * vec4(step_uv[0],1.0)).z;
      //float es_shift = 0.0;
      //float es_shift_scale = 2.0;

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        uv_coords        = uv_multiplier.yz;
        vec4 q_pos_ms                   = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position                     = kinect_mvp_matrix * q_pos_ms;

        float es_linear_depth_corner = (kinect_mv_matrix * q_pos_ms).z;

        //es_shift       = abs(es_linear_depth_corner - es_linear_depth_center);// * es_shift_scale;
        gl_Position.z  = ( ( -(es_linear_depth_corner /*+ es_shift*/ ) ) / gua_clip_far);
        gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
        gl_Position.z  *= gl_Position.w;
        
        //VertexOut.pass_es_linear_depth = (-es_linear_depth_corner) / gua_clip_far;

        //VertexOut.pass_es_shift = es_shift;
        EmitVertex();
      }

      EndPrimitive();
  //}



}



