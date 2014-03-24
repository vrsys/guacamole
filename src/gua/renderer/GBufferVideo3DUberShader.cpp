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
#include <gua/renderer/GBufferVideo3DUberShader.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases.hpp>
#include <gua/utils/logger.hpp>
#include <gua/memory.hpp>
#include <gua/renderer/GBuffer.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/renderer/FrameBufferObject.hpp>
#include <gua/renderer/Window.hpp>

#include <scm/gl_core/render_device/context_guards.h>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

void GBufferVideo3DUberShader::create(std::set<std::string> const& material_names) {

  UberShaderFactory vshader_factory(
    ShadingModel::GBUFFER_VERTEX_STAGE, material_names
  );

  UberShaderFactory fshader_factory(
    ShadingModel::GBUFFER_FRAGMENT_STAGE, material_names,
    vshader_factory.get_uniform_mapping()
  );

  LayerMapping vshader_output_mapping = vshader_factory.get_output_mapping();

  fshader_factory.add_inputs_to_main_functions(
    {&vshader_output_mapping}, ShadingModel::GBUFFER_VERTEX_STAGE
  );

  UberShader::set_uniform_mapping(fshader_factory.get_uniform_mapping());
  UberShader::set_output_mapping(fshader_factory.get_output_mapping());

  // create depth shader
  std::vector<ShaderProgramStage> warp_pass_stages;
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _warp_pass_vertex_shader()));
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_GEOMETRY_SHADER,        _warp_pass_geometry_shader()));
  warp_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _warp_pass_fragment_shader()));

  auto warp_pass_program = std::make_shared<ShaderProgram>();
  warp_pass_program->set_shaders(warp_pass_stages);
  warp_pass_program->save_to_file(".", "depth_pass");
  add_pre_pass(warp_pass_program);

  // create final shader
  std::vector<ShaderProgramStage> blend_pass_stages;
  blend_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_VERTEX_SHADER,          _blend_pass_vertex_shader(vshader_factory, vshader_output_mapping)));
  blend_pass_stages.push_back( ShaderProgramStage( scm::gl::STAGE_FRAGMENT_SHADER,        _blend_pass_fragment_shader(fshader_factory, vshader_output_mapping)));

  // generate shader source
  set_shaders ( blend_pass_stages );
  save_to_file(".", "final_pass");
}

std::string const GBufferVideo3DUberShader::_warp_pass_vertex_shader() const
{
  return R"(
    // header
    #version 420
    #extension GL_NV_bindless_texture : require
    #extension GL_NV_gpu_shader5      : enable
    #extension GL_ARB_explicit_uniform_location : enable
    #extension GL_ARB_shading_language_420pack : enable
    #extension GL_EXT_TEXTURE_ARRAY : enable

    // input -----------------------------------------------------------------------
    layout(location=0) in vec3 gua_in_position;
    layout(location=1) in vec2 gua_in_texcoords;
    layout(location=2) in vec3 gua_in_normal;
    layout(location=3) in vec3 gua_in_tangent;
    layout(location=4) in vec3 gua_in_bitangent;

    // uniforms //=========> @ include something bla
    uniform mat4  gua_projection_matrix;
    uniform mat4  gua_view_matrix;
    uniform mat4  gua_model_matrix;
    uniform mat4  gua_normal_matrix;
    uniform mat4  gua_inverse_projection_view_matrix;
    uniform vec3  gua_camera_position;

    //calibration matrices
    uniform mat4  image_d_to_eye_d;
    uniform mat4  eye_d_to_world;
    uniform mat4  eye_d_to_eye_rgb;
    uniform mat4  eye_rgb_to_image_rgb;

    //kinect depths
    uniform sampler2DArray depth_video3d_texture;
    uniform int layer;

    // outputs ---------------------------------------------------------------------
    out VertexData {
      vec2 texture_coord;
      vec3 pos_es;
      vec3 pos_d;
      vec3 pos_ws;
      float depth;
    } VertexOut;

    // main ------------------------------------------------------------------------
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
    )";
}

std::string const GBufferVideo3DUberShader::_warp_pass_geometry_shader() const
{ 
#if 0
  std::string geometry_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_geom) //TODO gbuffer_video3d_video3d_vert
  );
#else
  std::string geometry_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_TEXTURE_ARRAY : enable

layout (triangles) in;
layout (triangle_strip, max_vertices=3) out;

// constants
float min_length = 0.0125; //TODO uniform
float geo_length = 0.01; //TODO uniform

// gua uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;

// video uniforms
uniform int layer;

// input
in VertexData {
    vec2 texture_coord;
    vec3 pos_es;
    vec3 pos_d;
    vec3 pos_ws;
    float depth;
} VertexIn[3];

// output
out vec2  texture_coord;
out vec3  pos_es;
out vec3  pos_d;
out vec3  pos_ws;
out float depth;

// methods 
bool validSurface(vec3 a, vec3 b, vec3 c,
                  float depth_a, float depth_b, float depth_c)
{
  float avg_depth = (depth_a + depth_b + depth_c)/3.0;
  float baselength = 0.005;
  float l = min_length * avg_depth + baselength;

  if((length(a) > l) || (length(b) > l) || (length(c) > l)){
    return false;
  }

  if(depth_a < 0.1 || depth_b < 0.1 || depth_c < 0.1)
  {
        return false;
  }

  return true;
}


void main()
{
  vec3 a = VertexIn[1].pos_es - VertexIn[0].pos_es;
  vec3 b = VertexIn[2].pos_es - VertexIn[0].pos_es;
  vec3 c = VertexIn[2].pos_es - VertexIn[1].pos_es;

  float depth_a = VertexIn[0].depth;
  float depth_b = VertexIn[1].depth;
  float depth_c = VertexIn[2].depth;

  bool valid = validSurface(a, b, c, depth_a, depth_b, depth_c);

  if (valid)
  {      
      for(int i = 0; i < gl_in.length(); i++)
      {
        texture_coord = VertexIn[i].texture_coord;
        pos_es        = VertexIn[i].pos_es;
        pos_d         = VertexIn[i].pos_d;
        pos_ws        = VertexIn[i].pos_ws;
        depth         = VertexIn[i].depth;

        gl_Position   = gl_in[i].gl_Position;

        EmitVertex();
      }
      EndPrimitive();
  }
}
  )";
#endif

  return geometry_shader;
}

std::string const GBufferVideo3DUberShader::_warp_pass_fragment_shader() const
{
#if 0

  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_frag)//TODO gbuffer_video3d_video3d_frag
    );

#else

  std::string fragment_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_TEXTURE_ARRAY : enable

// gua uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;

// video uniforms
uniform int layer;
uniform int bbxclip;

// input
in vec2 texture_coord;
in vec3 pos_es;
in vec3 pos_d;
in vec3 pos_ws;
in float depth;

// output
layout (location=0) out vec4 out_color;

// methods 
vec3 bbx_min = vec3(-1.5,-0.1, -1.0);
vec3 bbx_max = vec3( 1.5,2.2,   1.5);

bool clip(vec3 p){
  if(p.x < bbx_min.x ||
     p.y < bbx_min.y ||
     p.z < bbx_min.z ||
     p.x > bbx_max.x ||
     p.y > bbx_max.y ||
     p.z > bbx_max.z){
    return true;
  }
  return false;
}

void main() 
{
#if 1
  if(clip(pos_ws) && bbxclip > 0)
    discard;
#endif

  // caclulate adhoc normal from ddepth
  vec3 a_d = dFdx(pos_d);
  vec3 b_d = -dFdy(pos_d);
  vec3 normal_d = normalize(cross(b_d,a_d));
  float d_angle = dot(normalize(-pos_d),normal_d);

#if 1
  // back face culling
  if(d_angle < 0.075) {
     discard;
  }
#endif

#if 1
   // to cull away borders of the rgb camera view
   if(texture_coord.s > 0.975 || texture_coord.s < 0.025 ||
      texture_coord.t > 0.975 || texture_coord.t < 0.025) {
        discard;
   }
#endif

  vec3 pos_d_to_depth_camera = normalize(pos_d);
  pos_d_to_depth_camera.x = pos_d_to_depth_camera.x;
  pos_d_to_depth_camera.y = pos_d_to_depth_camera.y;
  pos_d_to_depth_camera.z = -pos_d_to_depth_camera.z;

  float cosphi = dot(normal_d, pos_d_to_depth_camera);
  float depth_squared = (depth * depth);
  float quality = 1000.0;

  if(depth_squared > 0.0)
    quality = cosphi / depth_squared;
  else
    quality = 0.0;

  quality = quality * quality;

  float dist_es = length(pos_es);

  out_color = vec4(texture_coord, dist_es, quality);
}
)";

#endif

  return fragment_shader;
}

std::string const GBufferVideo3DUberShader::_blend_pass_vertex_shader(UberShaderFactory const& vshader_factory, LayerMapping const& vshader_output_mapping) const
{
#if 0 
  std::string vertex_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_vert) //TODO gbuffer_video3d_video3d_vert
  );
#else 
std::string vertex_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_TEXTURE_ARRAY : enable

// input
layout(location=0) in vec3 gua_in_position;
layout(location=2) in vec2 gua_in_texcoord;

// uniforms
uniform mat4  gua_projection_matrix;
uniform mat4  gua_view_matrix;
uniform mat4  gua_model_matrix;
uniform mat4  gua_normal_matrix;
uniform mat4  gua_inverse_projection_view_matrix;
uniform vec3  gua_camera_position;

uniform uint gua_material_id;

// material specific uniforms
@uniform_definition

// output
out vec3 gua_position_varying;
out vec2 gua_quad_coords;

// varying output
vec2 gua_texcoords;

vec3 gua_world_normal;
vec3 gua_world_position;
vec3 gua_world_tangent;
vec3 gua_world_bitangent;

vec3 gua_object_normal;
vec3 gua_object_position;
vec2 gua_object_texcoords;
vec3 gua_object_tangent;
vec3 gua_object_bitangent;

@output_definition

uint gua_get_material_id() {            
  return gua_uint_gbuffer_varying_0.x;
}  

@material_methods

// body
void main() {
  gua_position_varying = vec3(0);
  gua_quad_coords = gua_in_texcoord;
  gua_texcoords = gua_in_texcoord;

  gua_object_normal =     vec3(0);
  gua_object_tangent =    vec3(0);
  gua_object_bitangent =  vec3(0);
  gua_object_position =   vec3(0);

  gua_world_normal =      vec3(0);
  gua_world_tangent =     vec3(0);
  gua_world_bitangent =   vec3(0);
  gua_world_position =    vec3(0);

  // big switch, one case for each material
  @material_switch

  gua_uint_gbuffer_varying_0.x = gua_material_id;
  gl_Position = vec4(gua_in_position, 1.0);
}
)";
#endif

// material specific uniforms
string_utils::replace(vertex_shader, "@uniform_definition",
  get_uniform_mapping()->get_uniform_definition());

// output
string_utils::replace(vertex_shader, "@output_definition",
  vshader_output_mapping.get_gbuffer_output_definition(false, true));

// print material specific methods
string_utils::replace(vertex_shader, "@material_methods",
  UberShader::print_material_methods(vshader_factory));

// print main switch(es)
string_utils::replace(vertex_shader, "@material_switch",
  UberShader::print_material_switch(vshader_factory));

  return vertex_shader;
}

std::string const GBufferVideo3DUberShader::_blend_pass_fragment_shader(UberShaderFactory const& fshader_factory, LayerMapping const& vshader_output_mapping) const
{

#if 0

  std::string fragment_shader(
    Resources::lookup_shader(Resources::shaders_uber_shaders_gbuffer_video3d_video3d_frag)//TODO gbuffer_video3d_video3d_frag
    );

#else

  // todo : make Video3D material a ressource!
  std::string path_to_shading_model;
  std::string const video3d_shading_model_name;
  for (auto shading_model_name : ShadingModelDatabase::instance()->list_all())
  {
    auto video_shading_model_found = std::string::npos != shading_model_name.find("Video3D.gsd");
    if (video_shading_model_found) {
      path_to_shading_model = shading_model_name;
    }
  }

  if (path_to_shading_model.empty())
  {
    Logger::LOG_WARNING << "Could not find shading model " << video3d_shading_model_name.c_str() << " : File does not exist!" << std::endl;
  }

std::string fragment_shader = R"(

// header
#version 420
#extension GL_NV_bindless_texture : require
#extension GL_NV_gpu_shader5      : enable
#extension GL_ARB_explicit_uniform_location : enable
#extension GL_ARB_shading_language_420pack : enable
#extension GL_EXT_TEXTURE_ARRAY : enable

in vec3 gua_position_varying;
in vec2 gua_quad_coords;

// video3D
#define MAX_VIEWS 8
uniform int   numlayers;
uniform float epsilon;

uniform sampler2DArray depth_texture;
uniform sampler2DArray quality_texture;
uniform sampler2DArray video_color_texture;

uniform mat4 gua_inverse_projection_view_matrix;

isampler2D gua_get_int_sampler(uvec2 handle) {
    return isampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler2D gua_get_uint_sampler(uvec2 handle) {
    return usampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_float_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2D gua_get_double_sampler(uvec2 handle) {
    return sampler2D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler2DShadow  gua_get_shadow_sampler(uvec2 handle) {
    return sampler2DShadow(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

isampler3D gua_get_int_sampler3D(uvec2 handle) {
    return isampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

usampler3D gua_get_uint_sampler3D(uvec2 handle) {
    return usampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_float_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

sampler3D gua_get_double_sampler3D(uvec2 handle) {
    return sampler3D(uint64_t(handle.x) | (uint64_t(handle.y) << 32UL));
}

@input_definition

@uniform_definition

@output_definition

uint gua_get_material_id() {            
  return gua_uint_gbuffer_varying_0.x;
}  

vec3 gua_get_position() {
  return gua_position_varying;
}

// material specific methods
@material_methods



void main() {

  vec3  output_color = vec3(0.0);
  float output_depth = 1.0f;

  vec3 coords = vec3(gua_quad_coords, 0.0);

  vec4 color_contributions[MAX_VIEWS];

  float maxdist = 1000.0;
  float mindist = maxdist;
  float ogldepth = 1.0;

  // find minimum distances;
  for(int l = 0; l  < numlayers && l < MAX_VIEWS;++l)
  {
    coords.z = float(l);
    vec4 color_contribution = texture2DArray( quality_texture, coords);
    float depth             = texture2DArray( depth_texture, coords).r;

    vec4 p_os = gua_inverse_projection_view_matrix * vec4(gua_quad_coords.xy * 2.0f - vec2(1.0), depth*2.0f - 1.0f, 1.0);
    p_os = p_os / p_os.w;
    color_contribution.b = length(p_os.xyz);

    float dist = color_contribution.b;
    

    if(dist < maxdist && depth < 1.0)
    {
      

      color_contributions[l] = color_contribution;
      if(dist < mindist){
      	mindist = dist;
      	ogldepth = depth;
      }
    } else {
      color_contributions[l] = vec4(1.0,1.0,maxdist,1.0);
    }
  }

  int accum = 0;
  if(mindist < maxdist)  // we found at least one surface
  { 
    vec4 finalcol = vec4(0.0);

    for(int l = 0; l  < numlayers && l < MAX_VIEWS;++l)
    {
      if( abs(color_contributions[l].b - mindist) < epsilon)
      {
	      ++accum;
	      vec4 color_contribution = color_contributions[l];
	      vec4 color = texture2DArray( video_color_texture, vec3(color_contribution.xy,float(l)));
	      finalcol.rgb += color.rgb * color_contribution.a;
	      finalcol.a   += color_contribution.a;
      }
    }
    if(finalcol.a > 0.0) 
    {
      finalcol.rgba = finalcol.rgba/finalcol.a;
      output_depth = ogldepth;
      output_color = finalcol.rgb;
    } else {
      discard;
    }
  } else {
    discard;
  }

  // big switch, one case for each material
  @material_switch

  gua_uint_gbuffer_out_0.x = gua_uint_gbuffer_varying_0.x;

)";
fragment_shader += fshader_factory.get_output_mapping().get_output_string(path_to_shading_model, "video_color");
fragment_shader += R"( = output_color;
  gl_FragDepth = output_depth;
}
)";

#endif

  // input from vertex shader
  string_utils::replace(fragment_shader, "@input_definition",
    vshader_output_mapping.get_gbuffer_output_definition(true, true));

  // material specific uniforms
  string_utils::replace(fragment_shader, "@uniform_definition",
    get_uniform_mapping()->get_uniform_definition());

  // outputs
  string_utils::replace(fragment_shader, "@output_definition",
    get_gbuffer_mapping()->get_gbuffer_output_definition(false, false));

  // print material specific methods
  string_utils::replace(fragment_shader, "@material_methods",
    UberShader::print_material_methods(fshader_factory));

  // print main switch(es)
  string_utils::replace(fragment_shader, "@material_switch",
    UberShader::print_material_switch(fshader_factory));

  return fragment_shader;
}

////////////////////////////////////////////////////////////////////////////////
bool GBufferVideo3DUberShader::upload_to (RenderContext const& context) const
{
  assert(context.render_window->config.get_stereo_mode() == StereoMode::MONO ||
    ((context.render_window->config.get_left_resolution()[0] == context.render_window->config.get_right_resolution()[0]) &&
    (context.render_window->config.get_left_resolution()[1] == context.render_window->config.get_right_resolution()[1])));

  // initialize Texture Arrays (kinect depths & colors)
  if ( context.id >= warp_color_result_.size() ||
       context.id >= warp_depth_result_.size()) 
  {
    warp_depth_result_.resize(context.id + 1);
    warp_color_result_.resize(context.id + 1);

    warp_color_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_RGBA_32F,
      1,
      MAX_NUM_KINECTS,
      1
      );

    warp_depth_result_[context.id] = context.render_device->create_texture_2d(
      context.render_window->config.get_left_resolution(),
      scm::gl::FORMAT_D32F,
      1,
      MAX_NUM_KINECTS,
      1
      );
  }

  if (context.id >= fullscreen_quad_.size()) {
    fullscreen_quad_.resize(context.id + 1);
    fullscreen_quad_[context.id].reset(new scm::gl::quad_geometry(context.render_device, math::vec2(-1.f, -1.f), math::vec2(1.f, 1.f)));
  }

  if (context.id >= warp_result_fbo_.size()) {
    warp_result_fbo_.resize(context.id + 1);
    warp_result_fbo_[context.id] = context.render_device->create_frame_buffer();
  }

  if (context.id >= linear_sampler_state_.size()) {
    linear_sampler_state_.resize(context.id + 1);
    linear_sampler_state_[context.id] = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE);
  }

  if (context.id >= nearest_sampler_state_.size()) {
    nearest_sampler_state_.resize(context.id + 1);
    nearest_sampler_state_[context.id] = context.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
  }

  if (context.id >= depth_stencil_state_warp_pass_.size()) {
    depth_stencil_state_warp_pass_.resize(context.id + 1);
    depth_stencil_state_warp_pass_[context.id] = context.render_device->create_depth_stencil_state(false, true, scm::gl::COMPARISON_NEVER);
  }

  if (context.id >= depth_stencil_state_blend_pass_.size()) {
    depth_stencil_state_blend_pass_.resize(context.id + 1);
    depth_stencil_state_blend_pass_[context.id] = context.render_device->create_depth_stencil_state(false, true, scm::gl::COMPARISON_NEVER);
  }
  
  // upload base class programs & upload depth program
  bool warp_program_uploaded = get_pre_passes()[0]->upload_to(context);
  bool blend_program_uploaded = UberShader::upload_to(context);

  return warp_program_uploaded && blend_program_uploaded;
}

////////////////////////////////////////////////////////////////////////////////
void GBufferVideo3DUberShader::draw(RenderContext const& ctx,
                                    std::string const& ksfile_name,
                                    std::string const& material_name,
                                    scm::math::mat4 const& model_matrix,
                                    scm::math::mat4 const& normal_matrix) const
{
  // get render ressources
  auto video3d_ressource = Video3DDatabase::instance()->lookup(ksfile_name);
  auto material = MaterialDatabase::instance()->lookup(material_name);

  video3d_ressource->update_buffers(ctx);
  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    ctx.render_context->bind_texture(video3d_ressource->depth_array(ctx), nearest_sampler_state_[ctx.id], 0);
    get_pre_passes()[0]->get_program(ctx)->uniform_sampler("depth_video3d_texture", 0);

    auto ds_state = ctx.render_device->create_depth_stencil_state(true, true, scm::gl::COMPARISON_LESS);
    ctx.render_context->set_depth_stencil_state(ds_state);

    // pre passes
    for (unsigned layer = 0; layer != video3d_ressource->number_of_cameras(); ++layer)
    {
      // configure fbo
      warp_result_fbo_[ctx.id]->clear_attachments();
      warp_result_fbo_[ctx.id]->attach_depth_stencil_buffer(warp_depth_result_[ctx.id], 0, layer);
      warp_result_fbo_[ctx.id]->attach_color_buffer(0, warp_color_result_[ctx.id], 0, layer);
      
      // bind and clear fbo
      ctx.render_context->set_frame_buffer(warp_result_fbo_[ctx.id]);
      ctx.render_context->clear_depth_stencil_buffer(warp_result_fbo_[ctx.id]);
      ctx.render_context->clear_color_buffer(warp_result_fbo_[ctx.id], 0, scm::math::vec4f(0.0f, 0.0f, 0.0f, 0.0f));
       
      // set uniforms
      get_pre_passes()[0]->set_uniform(ctx, int(layer), "layer");
      get_pre_passes()[0]->set_uniform(ctx, normal_matrix, "gua_normal_matrix");
      get_pre_passes()[0]->set_uniform(ctx, model_matrix, "gua_model_matrix");

      if (material && video3d_ressource)
      {
        get_pre_passes()[0]->set_uniform(ctx, video3d_ressource->calibration_file(layer).getImageDToEyeD(), "image_d_to_eye_d");
        get_pre_passes()[0]->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeDToWorld(), "eye_d_to_world");
        get_pre_passes()[0]->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeDToEyeRGB(), "eye_d_to_eye_rgb");
        get_pre_passes()[0]->set_uniform(ctx, video3d_ressource->calibration_file(layer).getEyeRGBToImageRGB(), "eye_rgb_to_image_rgb");

        get_pre_passes()[0]->use(ctx);
        {
          video3d_ressource->draw(ctx);
        }
        get_pre_passes()[0]->unuse(ctx);
      }

      ctx.render_context->reset_framebuffer();
    }
  }

  {
    // single texture only
    scm::gl::context_all_guard guard(ctx.render_context);

    // second pass
    use(ctx);
    {
      if (material && video3d_ressource)
      {
        set_uniform(ctx, material->get_id(), "gua_material_id");
        set_uniform(ctx, normal_matrix, "gua_normal_matrix");
        set_uniform(ctx, model_matrix, "gua_model_matrix");

        // needs to be multiplied with scene scaling
        set_uniform(ctx, 0.075f, "epsilon");
        set_uniform(ctx, int(video3d_ressource->number_of_cameras()), "numlayers");

        ctx.render_context->bind_texture(warp_color_result_[ctx.id], nearest_sampler_state_[ctx.id], 0);
        programs_[ctx.id]->uniform_sampler("quality_texture", 0);

        ctx.render_context->bind_texture(warp_depth_result_[ctx.id], nearest_sampler_state_[ctx.id], 1);
        programs_[ctx.id]->uniform_sampler("depth_texture", 1);

        ctx.render_context->bind_texture(video3d_ressource->color_array(ctx), linear_sampler_state_[ctx.id], 2);
        programs_[ctx.id]->uniform_sampler("video_color_texture", 2);

        ctx.render_context->apply();
        fullscreen_quad_[ctx.id]->draw(ctx.render_context);
      }
    }
    unuse(ctx);
  }

  // restore depth stencil state
  //ctx.render_context->set_depth_stencil_state(old_depth_stencil_state);
}

}

