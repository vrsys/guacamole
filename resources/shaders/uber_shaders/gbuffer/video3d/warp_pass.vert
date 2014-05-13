@include "shaders/common/header.glsl"

///////////////////////////////////////////////////////////////////////////////
// input
///////////////////////////////////////////////////////////////////////////////
layout(location=0) in vec3 gua_in_position;
layout(location=1) in vec2 gua_in_texcoords;
layout(location=2) in vec3 gua_in_normal;
layout(location=3) in vec3 gua_in_tangent;
layout(location=4) in vec3 gua_in_bitangent;

///////////////////////////////////////////////////////////////////////////////
// uniforms
///////////////////////////////////////////////////////////////////////////////
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"

///////////////////////////////////////////////////////////////////////////////
// video3d uniforms
///////////////////////////////////////////////////////////////////////////////
uniform mat4  image_d_to_eye_d;
uniform mat4  eye_d_to_world;
uniform mat4  eye_d_to_eye_rgb;
uniform mat4  eye_rgb_to_image_rgb;

uniform sampler2DArray depth_video3d_texture;
uniform int layer;

///////////////////////////////////////////////////////////////////////////////
// outputs
///////////////////////////////////////////////////////////////////////////////
out VertexData {
    vec2 texture_coord;
    vec3 pos_es;
    vec3 pos_d;
    vec3 pos_ws;
    float depth;
} VertexOut;

///////////////////////////////////////////////////////////////////////////////
// main
///////////////////////////////////////////////////////////////////////////////
void main() 
{
    float depth             = texture2DArray(depth_video3d_texture, vec3(gua_in_texcoords.xy, layer)).r;

    vec4 POS_d              = depth * image_d_to_eye_d * vec4(gua_in_position.xy, depth, 1.0);
    POS_d.z                 = depth;
    POS_d.w                 = 1.0;
      
    vec4 POS_rgb            = eye_d_to_eye_rgb * POS_d;
    vec4 POS_ws             =  eye_d_to_world * POS_d;

    VertexOut.pos_d         = POS_d.xyz;
    VertexOut.pos_ws        = POS_ws.xyz;
    VertexOut.pos_es        = (gua_view_matrix * gua_model_matrix * POS_ws).xyz;
    VertexOut.texture_coord = (eye_rgb_to_image_rgb * vec4( (POS_rgb.xy/POS_rgb.z) ,0.0, 1.0)).xy;
    VertexOut.depth         = depth;

    gl_Position             = gua_projection_matrix * gua_view_matrix * gua_model_matrix * POS_ws;
}