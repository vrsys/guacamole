
// Copyright (c) 2012 Christopher Lux <christopherlux@gmail.com>
// Distributed under the Modified BSD License, see license.txt.

#version 420 core

#extension GL_ARB_shading_language_include : require
#extension GL_NV_gpu_shader5 : enable

#define VTEX_USE_FBACK_IMAGE							0
#define VTEX_USE_FBACK_LISTS							1
														
#define VTEX_VRM_VTEXTURE								0
#define VTEX_VRM_VTEXTURE_ATLAS							0
#define VTEX_VRM_VTEXTURE_PAGE_QUADRILIN				0
#define VTEX_VRM_VTEXTURE_PAGE_OCTREE					0
#define VTEX_VRM_RC_DVR_OCTREE							0
#define VTEX_VRM_RC_DVR_00								0
#define VTEX_VRM_RC_DVR_01								1
#define VTEX_VRM_BLEND_WEIGHTS							0
#define VTEX_VRM_RC_DVR_PREINT							0
#define VTEX_VRM_RC_DVR_ADJACENT_BLEND					0
#define VTEX_VRM_RC_DVR_ANIMATED_BLEND					0
#define VTEX_VRM_RC_DVR_GAUSSFIT						0
#define VTEX_VRM_RC_DVR_TEXTUREVIEW						0
#define VTEX_VRM_RC_DVR_LOD_ADAPTIVE_SAMPLING			1
#define VTEX_VRM_VTEXTURE_FIXED_LOD						0
#define VTEX_VRM_RC_OTREE_PCOORD						0
#define VTEX_VRM_RC_ITER_COUNT							0


//#include </scm/gl_util/camera_block.glslh>
// input/output definitions ///////////////////////////////////////////////////////////////////////
uniform uvec2 gua_depth_gbuffer_in;
uniform uvec2 gua_color_gbuffer_in;
uniform uvec2 gua_normal_gbuffer_in;
uniform uvec2 gua_ray_entry_in;
uniform uvec2 color_map;

uniform float uni_sampling_distance;
uniform vec3 uni_volume_bounds;

//////////////////////////////////////////???//!! PSEUDO UNIFORMS UGLY HACK
struct {                                   
        vec4 ws_position;               
        vec4 ws_near_plane;             
                                        
        mat4 v_matrix;                  
        mat4 v_matrix_inverse;          
                                        
        mat4 p_matrix;                  
                                         
        mat4 vp_matrix;                 
    } camera_transform; 

struct {
	vec4 volume_bbox_min;    // w unused
	vec4 volume_bbox_max;    // w unused
	vec4 scale_obj_to_tex;   // w unused
	vec4 sampling_distance;  // x - os sampling distance, y opacity correction factor, zw unused
	vec4 os_camera_position;
	vec4 value_range;        // vec4f(min_value(), max_value(), max_value() - min_value(), 1.0f / (max_value() - min_value()));

	mat4 m_matrix;
	mat4 m_matrix_transpose;
	mat4 m_matrix_inverse;

	//mat4 mv_matrix;

	mat4 mvp_matrix;
} volume_data;
//////////////////////////////////////////???//!! PSEUDO UNIFORMS UGLY HACK

#include </shaders/uber_shaders/common/get_sampler_casts.glsl>
#include </shaders/uber_shaders/common/gua_camera_uniforms.glsl>

#include </scm/data/ray/ray.glslh>
#include </scm/data/ray/ray_cast_result.glslh>
#include </scm/data/utilities/unproject.glslh>
		  
#include </scm/data/volume/ray_cast.glslh>
#include </scm/data/volume/ray_cast/uniforms.glslh>
#include </scm/data/volume/ray_cast/ray_cast_vtexture_3d_experimental.glslh>




//precision highp float;

// input layout definitions ///////////////////////////////////////////////////////////////////////
//layout(early_fragment_tests) in;

// global constants ///////////////////////////////////////////////////////////////////////////////



//in per_vertex {
//    smooth vec4 ray_entry_os;
//    smooth vec4 ray_entry_ts;
//} v_in;

// attribute layout definitions ///////////////////////////////////////////////////////////////////
// write outputs ---------------------------------------------------------------------
layout(location = 0) out vec3 gua_out_color;
//layout(depth_any)               out float gl_FragDepth;

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
  return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}




float get_depth_z(vec3 world_position) {
    vec4 pos = gua_projection_matrix * gua_view_matrix * vec4(world_position, 1.0);
    float ndc = pos.z/pos.w;
    return ((gl_DepthRange.diff * ndc) + gl_DepthRange.near + gl_DepthRange.far) / 2.0;

}

float get_depth_linear(float depth_buffer_d) {

	float ndc = (depth_buffer_d * 2.0 - gl_DepthRange.near - gl_DepthRange.far)/gl_DepthRange.diff;
	vec4 enit = vec4(gl_FragCoord.xy * 2.0 - vec2(1.0), ndc, 1.0);
	vec4 enit_inv = (gua_inverse_projection_view_matrix * enit);
	enit_inv /= enit_inv.w;
	return enit_inv.z;
}

vec3 get_object_world_position_from_depth(float depth_buffer_d) {

	float ndc = (depth_buffer_d * 2.0 - gl_DepthRange.near - gl_DepthRange.far) / gl_DepthRange.diff;
	vec4 enit = vec4(gl_FragCoord.xy * 2.0 - vec2(1.0), ndc, 1.0);
	vec4 enit_inv = (gua_inverse_projection_view_matrix * enit);	
	return enit_inv.xyz;
}

// implementation /////////////////////////////////////////////////////////////////////////////////
void main()
{
//////////////////////////////////////////???//!! PSEUDO UNIFORMS UGLY HACK

	vec3 gua_object_volume_position = texture2D(gua_get_float_sampler(gua_ray_entry_in), gua_get_quad_coords()).xyz;	
	///ANOTHER BAD HACK
	int volume_type_id = (int)texture2D(gua_get_float_sampler(gua_ray_entry_in), gua_get_quad_coords()).w;

	vec3 gua_world_volume_position = (gua_model_matrix * vec4(gua_object_volume_position, 1.0)).xyz;

	float d_gbuffer = texture2D(gua_get_float_sampler(gua_depth_gbuffer_in), gua_get_quad_coords()).x;
	float d_volume = get_depth_z(gua_world_volume_position);

	d_gbuffer = -1.0 * get_depth_linear(d_gbuffer);
	d_volume = -1.0 * get_depth_linear(d_volume);
	
	// compose  
	if (volume_type_id >= 0.9 && 		
		(gua_object_volume_position.x != 0.00 ||
		gua_object_volume_position.y != 0.00 ||
		gua_object_volume_position.z != 0.00))
	{


		camera_transform.ws_position = vec4(gua_camera_position, 1.0);
		camera_transform.ws_near_plane = vec4(gl_DepthRange.near);
		camera_transform.v_matrix = gua_view_matrix;
		camera_transform.v_matrix_inverse = inverse(gua_view_matrix);

		mat4 gua_invers_model_matrix = inverse(gua_model_matrix);

		volume_data.volume_bbox_min = vec4(0.0, 0.0, 0.0, 0.0);    // w unused
		volume_data.volume_bbox_max = vec4(uni_volume_bounds, 0.0);    // w unused
		volume_data.scale_obj_to_tex = vec4(vec3(1.0) / uni_volume_bounds, 1.0);   // w unused
		volume_data.sampling_distance = vec4(uni_sampling_distance, 1.0, uni_sampling_distance, 0.0);  // x - os sampling distance, y opacity correction factor, zw unused
		volume_data.os_camera_position = gua_invers_model_matrix * vec4(gua_camera_position, 1.0);
		volume_data.value_range = vec4(0.0, 1.0, 1.0, 1.0);        // vec4f(min_value(), max_value(), max_value() - min_value(), 1.0f / (max_value() - min_value()));

		volume_data.m_matrix = gua_model_matrix;
		volume_data.m_matrix_transpose = transpose(gua_model_matrix);
		volume_data.m_matrix_inverse = gua_invers_model_matrix;

		//volume_data.mv_matrix = gua_model_view_matrix;

		volume_data.mvp_matrix = gua_projection_matrix * gua_view_matrix * gua_model_matrix;

		//////////////////////////////////////////???//!! PSEUDO UNIFORMS UGLY HACK


		ray                 prim_r; // intersection ray
		vec2                t_span; // ray parameter interval
		float               sdist;  // sampling distance
		int                 i = 0; // iteration counter
		ray_cast_result     rc_res = make_ray_cast_result();
		
		vec3 ray_entry_os = texture2D(gua_get_float_sampler(gua_ray_entry_in), gua_get_quad_coords()).xyz;

		vvolume_ray_setup_ots(ray_entry_os.xyz, prim_r, t_span, sdist);

		vec3 gua_object_volume_position_front = (prim_r.origin + t_span.x * prim_r.direction) * 2.0;
		vec3 gua_world_volume_position_front = (gua_model_matrix * vec4(gua_object_volume_position_front, 1.0)).xyz;
		
		float d_volume_front = get_depth_z(gua_world_volume_position_front);		
		d_volume_front = -1.0 * get_depth_linear(d_volume_front);

		if (d_gbuffer < d_volume_front){
			gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_get_quad_coords()).xyz;
			return;
		}

		if (d_gbuffer < d_volume){ // there is geometry between inside			
			//float t_norm = t_span.y - t_span.x;
			float dg = d_gbuffer - d_volume_front;
			float dv = d_volume - d_volume_front;
			t_span.y -= (t_span.y - t_span.x) * 0.5;
			
			//ray_entry_os = get_object_world_position_from_depth(d_gbuffer);// , 1.0)).xyz;
			//vvolume_ray_setup_ots(ray_entry_os.xyz, prim_r, t_span, sdist);
		}

		vec4 out_color = vec4(0.0);
		//gua_out_color = texture(gua_get_float_sampler(color_map), gua_object_volume_position.xy).rgb;
		//gua_out_color = volume_data.volume_bbox_max.xyz;
		//return;

		if (t_span.x < t_span.y) {
			//#if VTEX_VRM_VTEXTURE == 1
			    //out_color = volume_draw_vtexture(prim_r, t_span, sdist, rc_res);
			//#elif VTEX_VRM_VTEXTURE_ATLAS == 1
			 //   out_color = volume_draw_debug_atlas(prim_r, t_span, sdist, rc_res);
			//#elif VTEX_VRM_VTEXTURE_PAGE_QUADRILIN == 1
			//    out_color = volume_draw_debug_page_quadrilinear(prim_r, t_span, sdist, rc_res);
			//#elif VTEX_VRM_VTEXTURE_PAGE_OCTREE == 1
			//    out_color = volume_draw_debug_quadtree(prim_r, t_span, sdist, rc_res);
			//#elif VTEX_VRM_RC_DVR_00 == 1 || VTEX_VRM_RC_DVR_01 == 1 || VTEX_VRM_RC_DVR_OCTREE == 1
			//#if VTEX_VRM_RC_DVR_ADJACENT_BLEND
			    out_color = volume_ray_cast_octree_exp(prim_r, t_span, sdist, rc_res);    
			//#else 
			//    out_color = volume_ray_cast_octree_blend_exp(prim_r, t_span, sdist, rc_res);
			//#endif
			//#endif
		}
#if VTEX_VRM_RC_ITER_COUNT == 1
		if (rc_res._iter_count > max_ray_iterations / 2) {
			out_color.rgb = red;
		}
		else {
			out_color.rgb = vec3(float(rc_res._iter_count) / 1024.0);
		}
		out_color.a = 1.0;
#endif // VTEX_VRM_RC_ITER_COUNT == 1

		//vec3 pos = t_span.y * prim_r.direction + prim_r.origin;
		//pos.rgb = pos.xyz /(vtex_volume.texcoord_scale * volume_data.scale_obj_to_tex.xyz);
		//out_color.rgb = pos.xyz;///= out_color.a;
		//out_color.rgb /= out_color.a;
		//out_color.a = 1.0;
		
		vec3 gbuffer_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_get_quad_coords()).xyz;
		gua_out_color = gbuffer_color * (1.0 - out_color.a) + out_color.rgb * out_color.a;
		//gua_out_color += vec3(0.0, 0.8, 0.0);

		//if (t_span.x < t_span.y) {
		//    // generate depth buffer value
		//    vec4 os_pos  = vec4(rc_res._pos / (vtex_volume.texcoord_scale * volume_data.scale_obj_to_tex.xyz), 1.0);
		//    vec4 cs_pos  = volume_data.mvp_matrix * os_pos;
		//    cs_pos      /= cs_pos.w;
		//    gl_FragDepth = clamp(cs_pos.z * 0.5 + 0.5, 0.0, 1.0);
		//}
		//else {
		//    //discard;
		//}
		//gua_out_color = gua_object_volume_position;

		//gua_out_color = gua_object_volume_position;
	}
	else
	{
		gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_get_quad_coords()).xyz;
	}
	
	//gua_out_color = texture(gua_get_float_sampler(color_map), gua_get_quad_coords()).rgb;// texture(gua_get_float_sampler(color_map), gua_object_volume_position.xy).rgb;
}

