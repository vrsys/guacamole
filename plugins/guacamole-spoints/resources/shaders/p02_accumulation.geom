@include "common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// general uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/common/gua_camera_uniforms.glsl"

in VertexDataIn {
  vec3 ms_pos;
  vec3 color;
  vec3 ms_u;
  vec3 ms_v;
  vec3 ms_normal;
} VertexIn[1];

out VertexDataOut {
  vec3 color;
  float log_depth;
  vec2 uv_coords;
} VertexOut;


//uniform mat4 kinect_model_matrix;
uniform mat4 kinect_mv_matrix;
uniform mat4 kinect_mvp_matrix;
uniform float point_size = 1.0;

uniform float voxel_half_size = 0.0;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////


layout (points) in;
layout (triangle_strip, max_vertices = 4) out;
//layout (points = 1 /*14*/) out;

const int ordered_line_strip_indices[14] = {3, 2, 6, 7, 4, 2, 0, 3, 1, 6,   5, 4, 1, 0};


void main() {

      // --------------------------- common attributes -----------------------------------
  VertexOut.color = (VertexIn[0].ms_normal+1.0) / 2.0;//VertexIn[0].color;

    
  vec4 object_normal = vec4(normalize(VertexIn[0].ms_normal), 0.0);
  vec4 view_normal  = inverse(transpose(kinect_mv_matrix)) * object_normal;

  if(true) {
  //if(view_normal.z > 0.0 ) {


      mat3x3 step_uv = mat3x3(gl_in[0].gl_Position.xyz,
                              VertexIn[0].ms_u,
                              VertexIn[0].ms_v);

      float es_linear_depth_center = (kinect_mv_matrix * vec4(step_uv[0],1.0)).z;
      //float es_shift = 0.0;
      //float es_shift_scale = 2.0;//2.0;

      const float index_arr[8] = {-1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0};

      // ---------------------------------------------------------------------------------
      for(int idx = 0; idx < 4; ++idx ) {
        vec3 uv_multiplier = vec3(1.0, 
                                  index_arr[idx],   
                                  index_arr[idx + 4]);

        VertexOut.uv_coords        = uv_multiplier.yz;
        vec4 q_pos_ms         = vec4( (step_uv * uv_multiplier) , 1.0);
        gl_Position           = kinect_mvp_matrix * q_pos_ms;
        VertexOut.log_depth        = (gl_Position.z/gl_Position.w)/2.0 + 0.5;
        
        float es_linear_depth_corner = (kinect_mv_matrix * q_pos_ms).z;

        //es_shift       = abs(es_linear_depth_corner - es_linear_depth_center);// * es_shift_scale;
        gl_Position.z  = ( ( -(es_linear_depth_corner /*+ es_shift*/ ) ) / gua_clip_far);
        gl_Position.z  = (gl_Position.z - 0.5) * 2.0;
        gl_Position.z  *= gl_Position.w;

        EmitVertex();
      }

      EndPrimitive();
      
  }
}



