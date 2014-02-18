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

@include "shaders/common/header.glsl"

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
#define VTEX_VRM_RC_DVR_ADJACENT_BLEND					1
#define VTEX_VRM_RC_DVR_ANIMATED_BLEND					0
#define VTEX_VRM_RC_DVR_GAUSSFIT						0
#define VTEX_VRM_RC_DVR_TEXTUREVIEW						0
#define VTEX_VRM_RC_DVR_LOD_ADAPTIVE_SAMPLING			1
#define VTEX_VRM_VTEXTURE_FIXED_LOD						0
#define VTEX_VRM_RC_OTREE_PCOORD						0
#define VTEX_VRM_RC_ITER_COUNT							0

// input from gbuffer ----------------------------------------------------
uniform uvec2 gua_depth_gbuffer_in;
uniform uvec2 gua_color_gbuffer_in;
uniform uvec2 gua_normal_gbuffer_in;
uniform uvec2 gua_ray_entry_in;

// uniforms
@include "shaders/uber_shaders/common/get_sampler_casts.glsl"
@include "shaders/uber_shaders/common/gua_camera_uniforms.glsl"
//uniforms volume
@include "shaders/uber_shaders/composite/volume_utils/uniforms.glslh"
@include "shaders/uber_shaders/composite/volume_utils/global_constants.glslh"

@include "shaders/uber_shaders/composite/volume_utils/utilities/unproject.glslh"
@include "shaders/uber_shaders/composite/volume_utils/utilities/float_utils.glslh"

@include "shaders/uber_shaders/composite/volume_utils/ray/ray.glslh"
@include "shaders/uber_shaders/composite/volume_utils/ray/intersect_utils.glslh"
@include "shaders/uber_shaders/composite/volume_utils/ray/ray_cast_result.glslh"

// methods ---------------------------------------------------------------------

// global gua_* methods
vec2 gua_get_quad_coords() {
    return vec2(gl_FragCoord.x * gua_texel_width, gl_FragCoord.y * gua_texel_height);
}

///VTEXTURE
#extension GL_ARB_shading_language_include : require
#include </scm/virtual_volume/ray_cast_vtexture_3d_experimental.glslh>
///VTEXTURE

@include "shaders/uber_shaders/composite/volume_utils/ray/ray_setup.glslh"
@include "shaders/uber_shaders/composite/volume_utils/raycast.glslh"


// write outputs ---------------------------------------------------------------------
layout(location=0) out vec3 gua_out_color;


// main ------------------------------------------------------------------------
void main() {

    int volume_type_id = (int)texture2D(gua_get_float_sampler(gua_ray_entry_in), gua_get_quad_coords()).w;



        //mat4 gua_inverse_view_matrix = inverse(gua_view_matrix);
        //mat4 gua_inverse_model_matrix = inverse(gua_model_matrix);
    if (volume_type_id > 0.1){
        ray prim_r; // intersection ray
        vec2 t_span; // ray parameter interval
        float sdist; // sampling distance
        int i = 0; // iteration counter

        ray_cast_result rc_res = make_ray_cast_result();

        vec3 gua_object_volume_position_back = texture2D(gua_get_float_sampler(gua_ray_entry_in), gua_get_quad_coords()).xyz;
        if (volume_type_id >= 0.9 && volume_type_id <= 1.1){
            volume_ray_setup_ots(gua_object_volume_position_back, prim_r, t_span, sdist);
        }

        if (volume_type_id >= 1.9 && volume_type_id <= 2.1){
            //needed for vtextures
            volume_data.scale_obj_to_tex = scale_obj_to_tex;
            volume_data.sampling_distance = sampling_distance;
            volume_data.value_range = vec4(value_range, 
                                        value_range.y - value_range.x, 
                                        1.0f / (value_range.y - value_range.x));

            vvolume_ray_setup_ots(gua_object_volume_position_back, prim_r, t_span, sdist);
        }

        vec4 compositing_color = vec4(0.0, 0.0, 0.0, 0.0);
        if (t_span.x < t_span.y) {
            
            if (volume_type_id >= 0.9 && volume_type_id <= 1.1){
                compositing_color = get_raycast_color(prim_r, t_span, sdist, rc_res);
            }

            if (volume_type_id >= 1.9 && volume_type_id <= 2.1){
                compositing_color = volume_ray_cast_octree_blend_exp(prim_r, t_span, sdist, rc_res);
            }
        }
        vec3 gbuffer_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_get_quad_coords()).xyz;

        gua_out_color = gbuffer_color * (1.0 - compositing_color.a) + compositing_color.rgb;
    }
    else{
        gua_out_color = texture2D(gua_get_float_sampler(gua_color_gbuffer_in), gua_get_quad_coords()).xyz;
    }
}

