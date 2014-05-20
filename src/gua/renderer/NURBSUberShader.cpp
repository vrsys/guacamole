/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

// class header
#include <gua/renderer/NURBSUberShader.hpp>

// guacamole headers
#include <gua/renderer/UberShaderFactory.hpp>
#include <gua/renderer/NURBSShader.hpp>
#include <gua/renderer/GuaMethodsFactory.hpp>
#include <gua/renderer/NURBSRessource.hpp>
#include <gua/databases.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <sstream>
#include <list>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void NURBSUberShader::create(std::set<std::string> const& material_names)
{
  UberShader::create(material_names);

  // create shader for predraw pass to pre-tesselate if necessary
  std::vector<ShaderProgramStage>   pre_shader_stages;
  std::list<std::string>            interleaved_stream_capture;

  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_VERTEX_SHADER, _transform_feedback_vertex_shader()));
  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_CONTROL_SHADER, _transform_feedback_tess_control_shader()));
  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_TESS_EVALUATION_SHADER, _transform_feedback_tess_evaluation_shader()));
  pre_shader_stages.push_back(ShaderProgramStage(scm::gl::STAGE_GEOMETRY_SHADER, _transform_feedback_geometry_shader()));

  interleaved_stream_capture.push_back("xfb_position");
  interleaved_stream_capture.push_back("xfb_index");
  interleaved_stream_capture.push_back("xfb_tesscoord");

  auto xfb_program = std::make_shared<ShaderProgram>();
  xfb_program->set_shaders(pre_shader_stages, interleaved_stream_capture, true);
  add_program(xfb_program);

  // create final ubershader that writes to gbuffer
  std::vector<ShaderProgramStage> shader_stages;
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _final_vertex_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_TESS_CONTROL_SHADER,    _final_tess_control_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_TESS_EVALUATION_SHADER, _final_tess_evaluation_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _final_geometry_shader()));
  shader_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _final_fragment_shader()));

  auto final_program = std::make_shared<ShaderProgram>();
  final_program->set_shaders(shader_stages);
  add_program(final_program);
}


////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_transform_feedback_vertex_shader () const
{
    std::stringstream tf_vertex;

    tf_vertex << std::string("                       \n\
        #version 420 core                            \n\
        #extension GL_NV_gpu_shader5      : enable   \n\
                                                     \n\
        // input attributes                          \n\
        layout (location = 0) in vec3  position;    \n\
        layout (location = 1) in uint  index;       \n\
        layout (location = 2) in vec4  tesscoord;   \n\
                                                     \n\
        // output attributes                         \n\
        flat out vec3  vPosition;                   \n\
        flat out uint   vIndex;                      \n\
        flat out vec2  vTessCoord;                  \n\
                                                     \n\
        void main()                                  \n\
        {                                            \n\
          vPosition  = position;                     \n\
          vIndex     = index;                        \n\
          vTessCoord = tesscoord.xy;                 \n\
        }                                            \n\
    ");

    return tf_vertex.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_transform_feedback_tess_control_shader () const
{
    std::stringstream tess_control;
    tess_control << std::string("                            \n\
                                                             \n\
        #version 420 core                                    \n\
        #extension GL_NV_gpu_shader5      : enable           \n\
                                                             \n\
        #define ID gl_InvocationID                           \n\
                                                             \n\
        layout(vertices = 4) out;                            \n\
                                                             \n\
        uniform samplerBuffer parameter_texture;             \n\
        uniform samplerBuffer attribute_texture;             \n\
                                                             \n\
        // uniforms                                          \n\
        uniform mat4 gua_projection_matrix;                  \n\
        uniform mat4 gua_view_matrix;                        \n\
        uniform mat4 gua_model_matrix;                       \n\
        uniform mat4 gua_normal_matrix;                      \n\
        uniform mat4 gua_inverse_projection_view_matrix;     \n\
        uniform vec3 gua_camera_position;                    \n\
                                                             \n\
        uniform float gua_texel_width;                       \n\
        uniform float gua_texel_height;                      \n\
        uniform float gua_tesselation_max_error;             \n\
                                                             \n\
        flat in vec3  vPosition[];                           \n\
        flat in uint  vIndex[];                              \n\
        flat in vec2  vTessCoord[];                          \n\
                                                             \n\
        flat out vec3 tcPosition[];                          \n\
        flat out uint tcIndex[];                             \n\
        flat out vec2 tcTessCoord[];                         \n\
                                                             \n\
    ");

    tess_control << NURBSShader::edge_length();
    tess_control << NURBSShader::control_polygon_length();
    tess_control << NURBSShader::edge_tess_level();
    tess_control << NURBSShader::is_inside();
    tess_control << NURBSShader::frustum_cull();

    tess_control << std::string("                                                         \n\
                                                                                          \n\
        void main()                                                                       \n\
        {                                                                                 \n\
          tcPosition[ID]  = vPosition[ID];                                                \n\
          tcIndex[ID]     = vIndex[ID];                                                   \n\
          tcTessCoord[ID] = vTessCoord[ID];                                               \n\
                                                                                          \n\
          mat4 mvp_matrix = gua_projection_matrix * gua_view_matrix * gua_model_matrix;   \n\
                                                                                          \n\
          // if ( frustum_cull ( mvp_matrix ) ) {                                         \n\
          if ( true )                                                                     \n\
          {                                                                               \n\
            const float max_tesselation_per_pass = 64.0f;                                 \n\
                                                                                          \n\
            vec4 data = texelFetch(attribute_texture, int(vIndex[ID]) * 5);               \n\
                                                                                          \n\
            uint surface_index   = floatBitsToUint(data.x);                               \n\
            uint surface_order_u = floatBitsToUint(data.y);                               \n\
            uint surface_order_v = floatBitsToUint(data.z);                               \n\
                                                                                          \n\
            vec4 edgelen = control_polygon_length(parameter_texture,                      \n\
                                                  mvp_matrix,                             \n\
                                                  int(surface_index),                     \n\
                                                  int(surface_order_u),                   \n\
                                                  int(surface_order_v),                   \n\
                                                  int(1.0f/gua_texel_width),              \n\
                                                  int(1.0f/gua_texel_height));            \n\
                                                                                          \n\
            //      3                                                                     \n\
            //  3------2                                                                  \n\
            //  |      |                                                                  \n\
            //0 |      | 2                                                                \n\
            //  |      |                                                                  \n\
            //  0------1                                                                  \n\
            //      1                                                                     \n\
                                                                                          \n\
            const float max_error_pre_tesselation = max_tesselation_per_pass *            \n\
                                                    gua_tesselation_max_error;            \n\
                                                                                          \n\
            float edge0 = edge_tesslevel(edgelen[0], max_error_pre_tesselation );         \n\
            float edge1 = edge_tesslevel(edgelen[1], max_error_pre_tesselation );         \n\
            float edge2 = edge_tesslevel(edgelen[2], max_error_pre_tesselation );         \n\
            float edge3 = edge_tesslevel(edgelen[3], max_error_pre_tesselation );         \n\
                                                                                          \n\
            float pre_tess_level = max(max(edge0, edge1), max(edge2, edge3));             \n\
                                                                                          \n\
            //Following three must be same for Ist Pass                                   \n\
            gl_TessLevelInner[0] = pre_tess_level;                                        \n\
            gl_TessLevelOuter[1] = pre_tess_level;                                        \n\
            gl_TessLevelOuter[3] = pre_tess_level;                                        \n\
                                                                                          \n\
            //Following three must be same for Ist Pass                                   \n\
            gl_TessLevelInner[1] = pre_tess_level;                                        \n\
            gl_TessLevelOuter[0] = pre_tess_level;                                        \n\
            gl_TessLevelOuter[2] = pre_tess_level;                                        \n\
                                                                                          \n\
          } else {                                                                        \n\
            // constant tesselation -> debugging only                                     \n\
            const float fixed_pre_tesselation = 2.0f;                                     \n\
                                                                                          \n\
            gl_TessLevelInner[0] = fixed_pre_tesselation;                                 \n\
            gl_TessLevelInner[1] = fixed_pre_tesselation;                                 \n\
                                                                                          \n\
            gl_TessLevelOuter[0] = fixed_pre_tesselation;                                 \n\
            gl_TessLevelOuter[1] = fixed_pre_tesselation;                                 \n\
            gl_TessLevelOuter[2] = fixed_pre_tesselation;                                 \n\
            gl_TessLevelOuter[3] = fixed_pre_tesselation;                                 \n\
          }                                                                               \n\
        }                                                                                 \n\
                                                                                          \n\
    ");

    return tess_control.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_transform_feedback_tess_evaluation_shader () const
{
    std::stringstream tess_eval;

    tess_eval << std::string("                               \n\
        #version 420 core                                    \n\
        #extension GL_NV_gpu_shader5      : enable           \n\
                                                             \n\
        layout(quads, equal_spacing, ccw) in;                \n\
                                                             \n\
        flat in vec3  tcPosition[];                          \n\
        flat in uint  tcIndex[];                             \n\
        flat in vec2  tcTessCoord[];                         \n\
                                                             \n\
        flat out vec3 tePosition;                            \n\
        flat out uint teIndex;                               \n\
        flat out vec2 teTessCoord;                           \n\
                                                             \n\
        // uniforms                                          \n\
        uniform mat4 gua_projection_matrix;                  \n\
        uniform mat4 gua_view_matrix;                        \n\
        uniform mat4 gua_model_matrix;                       \n\
        uniform mat4 gua_normal_matrix;                      \n\
        uniform mat4 gua_inverse_projection_view_matrix;     \n\
        uniform vec3 gua_camera_position;                    \n\
                                                             \n\
        uniform samplerBuffer parameter_texture;             \n\
        uniform samplerBuffer attribute_texture;             \n\
    ");

    tess_eval << NURBSShader::surface_horner_evaluation();

    tess_eval << std::string("                                                 \n\
                                                                               \n\
        void main()                                                            \n\
        {                                                                      \n\
          vec2 p1 = mix(tcTessCoord[0].xy, tcTessCoord[1].xy, gl_TessCoord.x); \n\
          vec2 p2 = mix(tcTessCoord[3].xy, tcTessCoord[2].xy, gl_TessCoord.x); \n\
          vec2 uv = clamp(mix(p1, p2, gl_TessCoord.y), 0.0, 1.0);              \n\
                                                                               \n\
          vec4 data = texelFetch(attribute_texture, int(tcIndex[0]) * 5);      \n\
          uint surface_index   = floatBitsToUint(data.x);                      \n\
          uint surface_order_u = floatBitsToUint(data.y);                      \n\
          uint surface_order_v = floatBitsToUint(data.z);                      \n\
                                                                               \n\
          vec4 puv, du, dv;                                                    \n\
          evaluateSurface(parameter_texture,                                   \n\
                          int(surface_index),                                  \n\
                          int(surface_order_u),                                \n\
                          int(surface_order_v),                                \n\
                          uv,                                                  \n\
                          puv);                                                \n\
                                                                               \n\
          tePosition  = puv.xyz;                                               \n\
          teIndex     = tcIndex[0];                                            \n\
          teTessCoord = uv;                                                    \n\
        }                                                                      \n\
    ");

    return tess_eval.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_transform_feedback_geometry_shader () const
{
   std::stringstream tf_geom;

   tf_geom << std::string("                                     \n\
        #version 420 core                                       \n\
        #extension GL_NV_gpu_shader5      : enable              \n\
                                                                \n\
        layout(triangles) in;                                   \n\
        layout(points, max_vertices = 4) out;                   \n\
                                                                \n\
        // in attributes                                        \n\
        flat in vec3  tePosition[3];                            \n\
        flat in uint  teIndex[3];                               \n\
        flat in vec2  teTessCoord[3];                           \n\
                                                                \n\
        // out per fragment                                     \n\
        layout (location = 0)       out vec3 xfb_position;      \n\
        layout (location = 1) flat  out uint xfb_index;         \n\
        layout (location = 2)       out vec2 xfb_tesscoord;     \n\
                                                                \n\
        uniform samplerBuffer parameter_texture;                \n\
        uniform samplerBuffer attribute_texture;                \n\
    ");

    tf_geom << NURBSShader::surface_horner_evaluation();

    tf_geom << std::string("                                                                   \n\
        void main()                                                                            \n\
        {                                                                                      \n\
            vec2 maxmax_tesscoord = max(max(teTessCoord[0], teTessCoord[1]), teTessCoord[2]);  \n\
            vec2 minmin_tesscoord = min(min(teTessCoord[0], teTessCoord[1]), teTessCoord[2]);  \n\
                                                                                               \n\
            vec2 minmax_tesscoord = vec2(minmin_tesscoord.x, maxmax_tesscoord.y);              \n\
            vec2 maxmin_tesscoord = vec2(maxmax_tesscoord.x, minmin_tesscoord.y);              \n\
                                                                                               \n\
            vec2 tesscoords[4];                                                                \n\
            tesscoords[0] = minmin_tesscoord;                                                  \n\
            tesscoords[1] = maxmin_tesscoord;                                                  \n\
            tesscoords[2] = maxmax_tesscoord;                                                  \n\
            tesscoords[3] = minmax_tesscoord;                                                  \n\
                                                                                               \n\
            int i, index;                                                                      \n\
            ivec4 order = ivec4(-1);                                                           \n\
                                                                                               \n\
            for ( i = 0; i <= 2; i++ )                                                         \n\
            {                                                                                  \n\
                bool minx = teTessCoord[i].x == minmin_tesscoord.x;                            \n\
                bool maxx = teTessCoord[i].x == maxmax_tesscoord.x;                            \n\
                bool miny = teTessCoord[i].y == minmin_tesscoord.y;                            \n\
                bool maxy = teTessCoord[i].y == maxmax_tesscoord.y;                            \n\
                                                                                               \n\
                int index = 2 * int(maxy) + int((minx && maxy) || (maxx && miny));             \n\
                                                                                               \n\
                order[index] = i;                                                              \n\
            }                                                                                  \n\
                                                                                               \n\
            // discard triangles                                                               \n\
            if ( order[3] == -1 || order[2] == -1 ) {                                          \n\
                return;                                                                        \n\
            }                                                                                  \n\
                                                                                               \n\
            vec2 new_tesscoord = (order[0] == -1) ?  minmin_tesscoord : maxmin_tesscoord;      \n\
                                                                                               \n\
            vec4 new_puv;                                                                      \n\
            vec4 new_du, new_dv;                                                               \n\
                                                                                               \n\
            vec4 data = texelFetch(attribute_texture, int(teIndex[0]) * 5);                    \n\
            uint surface_index   = floatBitsToUint(data.x);                                    \n\
            uint surface_order_u = floatBitsToUint(data.y);                                    \n\
            uint surface_order_v = floatBitsToUint(data.z);                                    \n\
                                                                                               \n\
            evaluateSurface ( parameter_texture,                                               \n\
                              int(surface_index),                                              \n\
                              int(surface_order_u),                                            \n\
                              int(surface_order_v),                                            \n\
                              new_tesscoord,                                                   \n\
                              new_puv );                                                       \n\
                                                                                               \n\
            for ( int i = 0; i != 4; ++i )                                                     \n\
            {                                                                                  \n\
                index         = order[i];                                                      \n\
                xfb_position 	= order[i] == -1 ? new_puv.xyz : tePosition[index];              \n\
                xfb_index 	  = teIndex[0];                                                    \n\
                xfb_tesscoord = tesscoords[i];                                                 \n\
                EmitVertex();                                                                  \n\
            }                                                                                  \n\
            EndPrimitive(); }                                                                  \n\
    ");

    return tf_geom.str();
}


////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_final_vertex_shader () const
{
    std::stringstream vertex_shader;

    vertex_shader << std::string("                  \n\
        #version 420 core                           \n\
        #extension GL_NV_gpu_shader5      : enable  \n\
                                                    \n\
        // hard-coded in attributes                 \n\
        layout (location = 0) in vec3  position;   \n\
        layout (location = 1) in uint  index;      \n\
        layout (location = 2) in vec2  tesscoord;  \n\
                                                    \n\
        // hard-coded output                        \n\
        flat out vec3  vPosition;                  \n\
        flat out uint  vIndex;                     \n\
        flat out vec2  vTessCoord;                 \n\
    ");

    vertex_shader << std::string("                 \n\
        void main()                                \n\
        {                                          \n\
            vPosition  = position;                \n\
            vIndex     = index;                   \n\
            vTessCoord = tesscoord;               \n\
        }                                          \n\
    ");

    return vertex_shader.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_final_tess_control_shader () const
{
    std::stringstream tess_ctrl;

    tess_ctrl << std::string("                              \n\
        #version 420 core                                   \n\
        #extension GL_NV_gpu_shader5      : enable          \n\
                                                            \n\
        #define ID gl_InvocationID                          \n\
                                                            \n\
        layout(vertices = 4) out;                           \n\
                                                            \n\
        uniform samplerBuffer parameter_texture;            \n\
        uniform samplerBuffer attribute_texture;            \n\
                                                            \n\
        uniform mat4 gua_projection_matrix;                 \n\
        uniform mat4 gua_view_matrix;                       \n\
        uniform mat4 gua_model_matrix;                      \n\
        uniform mat4 gua_normal_matrix;                     \n\
        uniform mat4 gua_inverse_projection_view_matrix;    \n\
        uniform vec3 gua_camera_position;                   \n\
        uniform float gua_tesselation_max_error;            \n\
                                                            \n\
                                                            \n\
        uniform float gua_texel_width;                      \n\
        uniform float gua_texel_height;                     \n\
                                                            \n\
        flat in vec3  vPosition[];                          \n\
        flat in uint  vIndex[];                             \n\
        flat in vec2  vTessCoord[];                         \n\
                                                            \n\
        flat out uint tcIndex[];                            \n\
        flat out vec2 tcTessCoord[];                        \n\
    ");

    tess_ctrl << NURBSShader::to_screen_space();
    tess_ctrl << NURBSShader::edge_length();
    tess_ctrl << NURBSShader::control_polygon_length();
    tess_ctrl << NURBSShader::surface_horner_evaluation();
    tess_ctrl << NURBSShader::edge_tess_level();
    tess_ctrl << NURBSShader::inner_tess_level();
    tess_ctrl << NURBSShader::is_inside();
    tess_ctrl << NURBSShader::frustum_cull();

    tess_ctrl << std::string("                                                                                                    \n\
                                                                                                                                  \n\
        void main()                                                                                                               \n\
        {                                                                                                                         \n\
          tcIndex[ID]     = vIndex[ID];                                                                                           \n\
          tcTessCoord[ID] = vTessCoord[ID];                                                                                       \n\
                                                                                                                                  \n\
          mat4 mvp_matrix = gua_projection_matrix * gua_view_matrix * gua_model_matrix;                                           \n\
                                                                                                                                  \n\
          vec4 data = texelFetch(attribute_texture, int(vIndex[ID]) * 5);                                                         \n\
          uint surface_index   = floatBitsToUint(data.x);                                                                         \n\
          uint surface_order_u = floatBitsToUint(data.y);                                                                         \n\
          uint surface_order_v = floatBitsToUint(data.z);                                                                         \n\
                                                                                                                                  \n\
          //if ( abs(vTessCoord[0].x - vTessCoord[1].x) * abs(vTessCoord[1].y - vTessCoord[2].y) == 1.0 )                         \n\
          if ( true )                                                                                                             \n\
          {                                                                                                                       \n\
            vec4 curve_factor = clamp(texelFetch(attribute_texture, int(vIndex[ID]) * 5 + 4), 1, 4);                              \n\
            vec4 edgelen = control_polygon_length ( parameter_texture,                                                            \n\
                                                    mvp_matrix,                                                                   \n\
                                                    int(surface_index),                                                           \n\
                                                    int(surface_order_u),                                                         \n\
                                                    int(surface_order_v),                                                         \n\
                                                    int(1.0f/gua_texel_width),                                                    \n\
                                                    int(1.0f/gua_texel_height) );                                                 \n\
                                                                                                                                  \n\
            //     3                                                                                                              \n\
            //  3------2                                                                                                          \n\
            //  |      |                                                                                                          \n\
            // 0|      |2                                                                                                         \n\
            //  |      |                                                                                                          \n\
            //  0------1                                                                                                          \n\
            //     1                                                                                                              \n\
                                                                                                                                  \n\
            float edge0 = edge_tesslevel(edgelen[0], gua_tesselation_max_error);                                                  \n\
            float edge1 = edge_tesslevel(edgelen[1], gua_tesselation_max_error);                                                  \n\
            float edge2 = edge_tesslevel(edgelen[2], gua_tesselation_max_error);                                                  \n\
            float edge3 = edge_tesslevel(edgelen[3], gua_tesselation_max_error);                                                  \n\
                                                                                                                                  \n\
            //Following three must be same for Ist Pass                                                                           \n\
            gl_TessLevelInner[0] = inner_tess_level(attribute_texture, int(vIndex[ID]) * 5,                                       \n\
                                                    mvp_matrix,                                                                   \n\
                                                    gua_tesselation_max_error,                                                    \n\
                                                    int(1.0f/gua_texel_width),                                                    \n\
                                                    int(1.0f/gua_texel_height));                                                  \n\
            gl_TessLevelOuter[1] = edge1;                                                                                         \n\
            gl_TessLevelOuter[3] = edge3;                                                                                         \n\
                                                                                                                                  \n\
            //Following three must be same for Ist Pass                                                                           \n\
            gl_TessLevelInner[1] = inner_tess_level(attribute_texture, int(vIndex[ID]) * 5,                                       \n\
                                                    mvp_matrix,                                                                   \n\
                                                    gua_tesselation_max_error,                                                    \n\
                                                    int(1.0f/gua_texel_width),                                                    \n\
                                                    int(1.0f/gua_texel_height));                                                  \n\
            gl_TessLevelOuter[0] = edge0;                                                                                         \n\
            gl_TessLevelOuter[2] = edge2;                                                                                         \n\
          } else {                                                                                                                \n\
                                                                                                                                  \n\
            int width  = int(1.0f/gua_texel_width);                                                                               \n\
            int height = int(1.0f/gua_texel_height);                                                                              \n\
                                                                                                                                  \n\
            vec4 point_on_plane0 = to_screen_space(vPosition[0], mvp_matrix, width, height);                                      \n\
            vec4 point_on_plane1 = to_screen_space(vPosition[1], mvp_matrix, width, height);                                      \n\
            vec4 point_on_plane2 = to_screen_space(vPosition[2], mvp_matrix, width, height);                                      \n\
            vec4 point_on_plane3 = to_screen_space(vPosition[3], mvp_matrix, width, height);                                      \n\
                                                                                                                                  \n\
            // Approach I ->                                                                                                      \n\
            // For Outer Tessellation Levels : Take ratio according to the original control polygon length.                       \n\
            // For Inner Tessellation Levels : Evaluate the mid point of the patch and get the diagonal length.                   \n\
                                                                                                                                  \n\
            vec4 edgelen = control_polygon_length(parameter_texture,                                                              \n\
                                                  mvp_matrix,                                                                     \n\
                                                  int(surface_index),                                                             \n\
                                                  int(surface_order_u),                                                           \n\
                                                  int(surface_order_v),                                                           \n\
                                                  width,                                                                          \n\
                                                  height);                                                                        \n\
                                                                                                                                  \n\
            vec2 p1 = mix(vTessCoord[0].xy, vTessCoord[1].xy, 0.5);                                                               \n\
            vec2 p2 = mix(vTessCoord[3].xy, vTessCoord[2].xy, 0.5);                                                               \n\
                                                                                                                                  \n\
            vec2 mid_uv = mix(p1, p2, 0.5);                                                                                       \n\
            vec4 mid_p;                                                                                                           \n\
                                                                                                                                  \n\
            evaluateSurface(parameter_texture,                                                                                    \n\
                            int(surface_index),                                                                                   \n\
                            int(surface_order_u),                                                                                 \n\
                            int(surface_order_v),                                                                                 \n\
                            mid_uv,                                                                                               \n\
                            mid_p);                                                                                               \n\
                                                                                                                                  \n\
            vec4 mid_p_screenspace = to_screen_space(mid_p.xyz, mvp_matrix, width, height);                                       \n\
                                                                                                                                  \n\
            float length1 = length(point_on_plane0.xy - mid_p_screenspace.xy) +                                                   \n\
                            length(mid_p_screenspace.xy - point_on_plane3.xy);                                                    \n\
                                                                                                                                  \n\
            float length2 = length(point_on_plane2.xy - mid_p_screenspace.xy) +                                                   \n\
                            length(mid_p_screenspace.xy - point_on_plane1.xy);                                                    \n\
                                                                                                                                  \n\
            float diagonal_length = min(length1, length2);                                                                        \n\
                                                                                                                                  \n\
            float edge01 = edge_tesslevel(mix(edgelen[0], edgelen[2], abs(vTessCoord[0].y - vTessCoord[2].y)), max_screen_error); \n\
            float edge32 = edge_tesslevel(mix(edgelen[0], edgelen[2], abs(vTessCoord[0].y - vTessCoord[2].y)), max_screen_error); \n\
            float edge13 = edge_tesslevel(mix(edgelen[1], edgelen[3], abs(vTessCoord[0].x - vTessCoord[1].x)), max_screen_error); \n\
            float edge20 = edge_tesslevel(mix(edgelen[1], edgelen[3], abs(vTessCoord[0].x - vTessCoord[1].x)), max_screen_error); \n\
                                                                                                                                  \n\
            // Approach II ->                                                                                                     \n\
            // For Outer Tessellation Levels : Approx. the curvature length of the edge according to the angle betw. its normals. \n\
            // For Inner Tessellation Levels : Approx. the curvature of the surface according to the all edge normals.            \n\
                                                                                                                                  \n\
            //Following three must be same for Ist Pass                                                                           \n\
            gl_TessLevelInner[0] = edge_tesslevel(diagonal_length, max_screen_error);                                             \n\
            gl_TessLevelOuter[1] = edge01;                                                                                        \n\
            gl_TessLevelOuter[3] = edge32;                                                                                        \n\
                                                                                                                                  \n\
            //Following three must be same for Ist Pass                                                                           \n\
            gl_TessLevelInner[1] = edge_tesslevel(diagonal_length, max_screen_error);                                             \n\
            gl_TessLevelOuter[0] = edge20;                                                                                        \n\
            gl_TessLevelOuter[2] = edge13;                                                                                        \n\
          }                                                                                                                       \n\
        }                                                                                                                         \n\
    ");

    return tess_ctrl.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_final_tess_evaluation_shader () const
{
    std::stringstream tess_eval;

    tess_eval << std::string("                                  \n\
        #version 420 core                                       \n\
        #extension GL_NV_gpu_shader5      : enable              \n\
                                                                \n\
        layout(quads, equal_spacing, ccw) in;                   \n\
                                                                \n\
        flat in uint  tcIndex[];                                \n\
        flat in vec2  tcTessCoord[];                            \n\
                                                                \n\
        flat out vec3   teBitangent;                            \n\
        flat out vec3   teTangent;                              \n\
        flat out uint   teIndex;                                \n\
        flat out vec2   teTessCoord;                            \n\
        flat out vec4   teNormal;                               \n\
        flat out vec4   tePosition;                             \n\
                                                                \n\
        // uniforms                                             \n\
        uniform mat4 gua_projection_matrix;                     \n\
        uniform mat4 gua_view_matrix;                           \n\
        uniform mat4 gua_model_matrix;                          \n\
        uniform mat4 gua_normal_matrix;                         \n\
        uniform mat4 gua_inverse_projection_view_matrix;        \n\
        uniform vec3 gua_camera_position;                       \n\
                                                                \n\
        uniform samplerBuffer parameter_texture;                \n\
        uniform samplerBuffer attribute_texture;                \n\
    ");

    tess_eval << NURBSShader::surface_horner_evaluation();

    tess_eval << std::string("                                                  \n\
                                                                                \n\
        void main()                                                             \n\
        {                                                                       \n\
          vec4 p, du, dv;                                                       \n\
                                                                                \n\
          vec4 data = texelFetch(attribute_texture, int(tcIndex[0]) * 5);       \n\
          uint surface_index   = floatBitsToUint(data.x);                       \n\
          uint surface_order_u = floatBitsToUint(data.y);                       \n\
          uint surface_order_v = floatBitsToUint(data.z);                       \n\
                                                                                \n\
          vec2 p1 = mix(tcTessCoord[0].xy, tcTessCoord[1].xy, gl_TessCoord.x);  \n\
          vec2 p2 = mix(tcTessCoord[3].xy, tcTessCoord[2].xy, gl_TessCoord.x);  \n\
                                                                                \n\
          vec2 uv;                                                              \n\
                                                                                \n\
          uv = clamp(mix(p1, p2, gl_TessCoord.y), 0.0, 1.0);                    \n\
                                                                                \n\
          evaluateSurface(parameter_texture,                                    \n\
                          int(surface_index),                                   \n\
                          int(surface_order_u),                                 \n\
                          int(surface_order_v),                                 \n\
                          uv, p, du, dv);                                       \n\
                                                                                \n\
          tePosition  = vec4(p.xyz, 1.0);                                       \n\
          teBitangent = normalize(du.xyz);                                      \n\
          teTangent   = normalize(dv.xyz);                                      \n\
          teIndex     = tcIndex[0];                                             \n\
          teTessCoord = uv;                                                     \n\
          teNormal    = vec4(normalize(cross(du.xyz, dv.xyz)), 0.0);            \n\
                                                                                \n\
          vec4 nview  = gua_normal_matrix * teNormal;                           \n\
          vec4 pview  = gua_view_matrix * gua_model_matrix * tePosition;        \n\
                                                                                \n\
          if ( dot(normalize(nview.xyz), -normalize(pview.xyz)) < 0.0f ) {      \n\
            teNormal = -teNormal;                                               \n\
          }                                                                     \n\
        }                                                                       \n\
    ");

    return tess_eval.str();
}


////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_final_geometry_shader () const
{
    std::stringstream geom_shader;

    geom_shader << std::string("                             \n\
        #version 420 core                                    \n\
        #extension GL_NV_bindless_texture : require          \n\
        #extension GL_NV_gpu_shader5      : enable           \n\
                                                             \n\
        layout(triangles) in;                                \n\
        layout(triangle_strip, max_vertices = 3) out;        \n\
                                                             \n\
        uniform mat4 gua_projection_matrix;                  \n\
        uniform mat4 gua_view_matrix;                        \n\
        uniform mat4 gua_model_matrix;                       \n\
        uniform mat4 gua_normal_matrix;                      \n\
        uniform mat4 gua_inverse_projection_view_matrix;     \n\
        uniform vec3 gua_camera_position;                    \n\
        uniform uint gua_material_id;                        \n\
                                                             \n\
        flat in vec3 teBitangent[3];                         \n\
        flat in vec3 teTangent[3];                           \n\
        flat in uint teIndex[3];                             \n\
        flat in vec2 teTessCoord[3];                         \n\
        flat in vec4 teNormal[3];                            \n\
        flat in vec4 tePosition[3];                          \n\
                                                             \n\
        flat out uint gIndex;                                \n\
        out vec2      gTessCoord;                            \n\
                                                             \n\
        uniform samplerBuffer parameter_texture;             \n\
        uniform samplerBuffer attribute_texture;             \n\
    ");

    // generated material-dependent uniform definitions
    geom_shader << fshader_factory_->get_uniform_mapping().get_uniform_definition();
    geom_shader << std::endl;

    // generated varying variables
    geom_shader << "out vec3 gua_position_varying;" << std::endl;
    geom_shader << vshader_factory_->get_output_mapping().get_gbuffer_output_definition(false, true);
    geom_shader << std::endl;

    // hard-coded buit-in output variables -> should be written!
    geom_shader << std::string("     \n\
        vec2 gua_texcoords;          \n\
                                     \n\
        vec3 gua_world_normal;       \n\
        vec3 gua_world_position;     \n\
        vec3 gua_world_tangent;      \n\
        vec3 gua_world_bitangent;    \n\
                                     \n\
        vec3 gua_object_normal;      \n\
        vec3 gua_object_position;    \n\
        vec3 gua_object_tangent;     \n\
        vec3 gua_object_bitangent;   \n\
    ");

    // generated auxiliary methods
    GuaMethodsFactory method_factory;
    geom_shader << method_factory.get_sampler_casts() << std::endl;

    // material specific methods -----------------------------------------------
    for (auto const& method : vshader_factory_->get_main_functions()) {
        geom_shader << method.second << std::endl;
    }

    geom_shader << std::string("                                                                               \n\
        void main()                                                                                            \n\
        {                                                                                                      \n\
            for ( int i = 0; i != 3; ++i )                                                                     \n\
            {                                                                                                  \n\
                gIndex      = teIndex[i];                                                                      \n\
                gTessCoord  = teTessCoord[i];                                                                  \n\
                                                                                                               \n\
                // write built-in input for material                                                           \n\
                ///////////////////////////////////////////////////////                                        \n\
                gua_texcoords  = gTessCoord;                                                                   \n\
                                                                                                               \n\
                gua_position_varying = (gua_model_matrix * tePosition[i]).xyz;                                 \n\
                gua_object_normal    = teNormal[i].xyz;                                                        \n\
                gua_object_tangent   = teTangent[i].xyz;                                                       \n\
                gua_object_bitangent = teBitangent[i].xyz;                                                     \n\
                gua_object_position  = tePosition[i].xyz;                                                      \n\
                                                                                                               \n\
                gua_world_normal     = normalize ( gua_normal_matrix * vec4 (teNormal[i].xyz, 0.0) ).xyz;      \n\
                gua_world_tangent    = normalize ( gua_normal_matrix * vec4 (teTangent[i].xyz, 0.0) ).xyz;     \n\
                gua_world_bitangent  = normalize ( gua_normal_matrix * vec4 (teBitangent[i].xyz, 0.0) ).xyz;   \n\
                gua_world_position   = (gua_model_matrix * tePosition[i]).xyz;                                 \n\
                ///////////////////////////////////////////////////////                                        \n\
    ");

    // generated code
    auto main_calls(vshader_factory_->get_main_calls());
    unsigned current_case(0);
    unsigned cases_per_block(CASES_PER_UBERSHADER_SWITCH);

    auto call(main_calls.begin());

    while (current_case < main_calls.size()) {

        geom_shader << "switch(gua_material_id) {" << std::endl;

        for (unsigned i(current_case); i < main_calls.size() && i < current_case+cases_per_block; ++i) {
            geom_shader << " case " << call->first << ": " << call->second << " break;" << std::endl;
            ++call;
        }

        geom_shader << "}" << std::endl;


        current_case += cases_per_block;
    }

    geom_shader << std::string("                                                                                     \n\
                        gua_uint_gbuffer_varying_0.x = gua_material_id;                                              \n\
                        gl_Position = gua_projection_matrix * gua_view_matrix * vec4(gua_position_varying.xyz, 1.0); \n\
                        EmitVertex();                                                                                \n\
                    }                                                                                                \n\
            EndPrimitive();                                                                                          \n\
        }                                                                                                            \n\
    ");

    return geom_shader.str();
}

////////////////////////////////////////////////////////////////////////////////
std::string const NURBSUberShader::_final_fragment_shader () const
{
    std::string fragment_shader;

    fragment_shader += R"(                          
        #version 420 core                           
        #extension GL_NV_bindless_texture : require 
        #extension GL_NV_gpu_shader5 : enable       
                                                    
        #define TRIM_ERROR_TOLERANCE 0.00001        
                                                    
        precision highp float;                      
              
        flat in uint gIndex;                         
        in vec2      gTessCoord;                     
                                                     
        uniform samplerBuffer attribute_texture;
                                                
        uniform samplerBuffer trim_partition;   
        uniform samplerBuffer trim_contourlist; 
        uniform samplerBuffer trim_curvelist;   
        uniform samplerBuffer trim_curvedata;   
        uniform samplerBuffer trim_pointdata;   
    )";

    fragment_shader += NURBSShader::curve_horner_evaluation();
    fragment_shader += NURBSShader::binary_search();
    fragment_shader += NURBSShader::bisect_curve();
    fragment_shader += NURBSShader::trimming_helper_methods();
    fragment_shader += NURBSShader::bisect_contour();
    fragment_shader += NURBSShader::contour_binary_search();
    fragment_shader += NURBSShader::contour_based_trimming();

    // input from vertex shader ------------------------------------------
    fragment_shader += R"(

        in vec3 gua_position_varying;
  
        //***** generated input defintion
        @input_definition            
                                  
        // uniforms                                      
        uniform mat4 gua_projection_matrix;              
        uniform mat4 gua_view_matrix;                    
        uniform mat4 gua_model_matrix;                   
        uniform mat4 gua_normal_matrix;                  
        uniform mat4 gua_inverse_projection_view_matrix; 
        uniform vec3 gua_camera_position;                
                                                         
        uniform float gua_texel_width;                   
        uniform float gua_texel_height;                  
           
        
        //***** generated uniform definition
        @uniform_definition 
        
        //***** generated output definition
        @output_definition                         
        
        // global gua_* methods                                                               
        vec2 gua_get_quad_coords() {                                                          
            return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height); 
        }                                                                                     
    )";

    GuaMethodsFactory method_factory;
    fragment_shader += method_factory.get_sampler_casts();

    fragment_shader += R"(
        uint gua_get_material_id() {            
            return gua_uint_gbuffer_varying_0.x;
        }                                       
                                                
        vec3 gua_get_position() {               
            return gua_position_varying;        
        }  

        //***** generated material methods
        @material_methods                                                            

        // main switch                                                                                  
        void main()                                                                                     
        {                                                                                               
            vec4 data = texelFetch(attribute_texture, int(gIndex) * 5);                                 
            uint trim_index = floatBitsToUint(data.w);                                                  
                                                                                                        
            vec4 nurbs_domain = texelFetch(attribute_texture, int(gIndex) * 5 + 1);                     
                                                                                                        
            vec2 domain_size  = vec2(nurbs_domain.z - nurbs_domain.x, nurbs_domain.w - nurbs_domain.y); 
                                                                                                        
            vec2 uv_nurbs     = gTessCoord.xy * domain_size + nurbs_domain.xy;                          
                                                                                                        
            int tmp = 0;                                                                                
            bool trimmed      = trim (trim_partition,                                                   
                                      trim_contourlist,                                                 
                                      trim_curvelist,                                                   
                                      trim_curvedata,                                                   
                                      trim_pointdata,                                                   
                                      uv_nurbs,                                                         
                                      int(trim_index), 1, tmp, 0.0001f, 16);                            
            if ( trimmed ) {                                                                            
                discard;                                                                                
            }  

          @material_switch

          gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;
        }                                                                                         
    )";

    // input from vertex shader
    string_utils::replace(fragment_shader, "@input_definition",
      vshader_factory_->get_output_mapping().get_gbuffer_output_definition(true, true));

    // material specific uniforms
    string_utils::replace(fragment_shader, "@uniform_definition",
      get_uniform_mapping()->get_uniform_definition());
    // BOOST_FOREACH (auto method, fragment_shader_factory_->get_main_functions()) {
    //     fragment_shader << method.second << std::endl;
    // }

    // outputs
    string_utils::replace(fragment_shader, "@output_definition",
      get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

    // print material specific methods
    string_utils::replace(fragment_shader, "@material_methods",
      UberShader::print_material_methods(*fshader_factory_));

    // print main switch(es)
    string_utils::replace(fragment_shader, "@material_switch",
      UberShader::print_material_switch(*fshader_factory_));

    return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ GeometryUberShader::stage_mask const NURBSUberShader::get_stage_mask() const
{
  return GeometryUberShader::DRAW_STAGE;
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::preframe(RenderContext const& ctx) const
{
  throw std::runtime_error("NURBSUberShader::preframe(): not implemented");
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::predraw(RenderContext const& ctx,
                                          std::string const& ksfile_name,
                                          std::string const& material_name,
                                          scm::math::mat4 const& model_matrix,
                                          scm::math::mat4 const& normal_matrix,
                                          Frustum const& /*frustum*/,
                                          std::size_t viewid) const
{
  throw std::runtime_error("NURBSUberShader::predraw(): not implemented");
}


////////////////////////////////////////////////////////////////////////////////

void NURBSUberShader::draw(RenderContext const& ctx,
                           std::string const& filename,
                           std::string const& material_name,
                           scm::math::mat4 const& model_matrix,
                           scm::math::mat4 const& normal_matrix,
                           Frustum const& /*frustum*/,
                           std::size_t /*viewid*/) const
{
  auto geometry = std::static_pointer_cast<NURBSRessource>(GeometryDatabase::instance()->lookup(filename));
  auto material = MaterialDatabase::instance()->lookup(material_name);

  if ( geometry && material )
  {
    set_uniform(ctx, 128, "gua_max_tesselation");
    set_uniform(ctx, material->get_id(), "gua_material_id");
    set_uniform(ctx, model_matrix, "gua_model_matrix");
    set_uniform(ctx, normal_matrix, "gua_normal_matrix");

  #ifdef DEBUG_XFB_OUTPUT
    scm::gl::transform_feedback_statistics_query_ptr q = ctx
      .render_device->create_transform_feedback_statistics_query(0);
    ctx.render_context->begin_query(q);
  #endif

    // pre-tesselate if necessary
    get_program(transform_feedback_pass)->use(ctx);
    {
      ctx.render_context->apply();
      geometry->predraw(ctx);
    }
    get_program(transform_feedback_pass)->unuse(ctx);

#ifdef DEBUG_XFB_OUTPUT
    ctx.render_context->end_query(q);
    ctx.render_context->collect_query_results(q);
    std::cout << q->result()._primitives_generated << " , "
      << q->result()._primitives_written << std::endl;
#endif

    // invoke tesselation/trim shader for adaptive nurbs rendering
    get_program(final_pass)->use(ctx);
    {
      ctx.render_context->apply();
      geometry->draw(ctx);
    }
    get_program(final_pass)->unuse(ctx);
  }
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::postdraw(RenderContext const& ctx,
  std::string const& ksfile_name,
  std::string const& material_name,
  scm::math::mat4 const& model_matrix,
  scm::math::mat4 const& normal_matrix,
  Frustum const& /*frustum*/,
  std::size_t /*viewid*/) const
{
  throw std::runtime_error("NURBSUberShader::postdraw(): not implemented");
}

////////////////////////////////////////////////////////////////////////////////

/*virtual*/ void NURBSUberShader::postframe(RenderContext const& ctx) const
{
  throw std::runtime_error("NURBSUberShader::postframe(): not implemented");
}


}
