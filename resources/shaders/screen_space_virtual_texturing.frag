@include "common/header.glsl"

// varying input
in vec2 gua_quad_coords;
// gbuffer input
@include "common/gua_camera_uniforms.glsl"
@include "common/gua_gbuffer_input.glsl"

@include "common/gua_fragment_shader_input.glsl"

layout (location = 0) out vec3 out_vt_color;

layout(binding = 0) uniform sampler2D gua_uv_buffer;
//layout(binding = 1) uniform sampler2DArray layered_physical_texture;

//layout(binding = 2) uniform usampler2D hierarchical_idx_textures;

struct padded_vt_address{
  uvec2 vt_address;
  ivec2 max_level_and_padding;
};

struct physical_texture_info_struct {
  uvec2 address;
  uvec2 tile_size;
  vec2  tile_padding;
  uvec2 dims;
};

layout(std430, binding = 0) buffer out_lod_feedback { int out_lod_feedback_values[]; };
layout(std430, binding = 1) buffer out_count_feedback { uint out_count_feedback_values[]; };

layout(std140, binding = 3) uniform physical_texture_address { physical_texture_info_struct pt; };
layout(std140, binding = 4) uniform virtual_texture_addresses { padded_vt_address vts[1024]; };


struct idx_tex_positions
{
    int parent_lvl;
    uvec4 parent_idx;
    int child_lvl;
    uvec4 child_idx;
};

const vec2 tile_padding_by_tile_size = pt.tile_padding / pt.tile_size;
const uint pt_slice_size = pt.dims.x * pt.dims.y;
/*
* Physical texture lookup
*/
vec4 get_physical_texture_color(uvec4 index_quadruple, vec2 texture_sampling_coordinates, uint current_level, int max_level)
{
    // exponent for calculating the occupied pixels in our index texture, based on which level the tile is in
    int max_level_minus_1 = max_level - 1;

    uint tile_occupation_exponent = (max_level_minus_1) - current_level;

    // 2^tile_occupation_exponent defines how many pixel (of the index texture) are used by the given tile
    uint occupied_index_pixel_per_dimension = uint(1 << tile_occupation_exponent);

    // offset represented as tiles is divided by total num tiles per axis
    // (replace max_width_tiles later by correct uniform)
    // extracting x,y from index texture
    uvec2 base_xy_offset = index_quadruple.xy;

    // base x,y coordinates * number of tiles / number of used index texture pixel
    // taking the factional part by modf
    vec2 physical_tile_ratio_xy = fract(texture_sampling_coordinates * (1 << (max_level_minus_1) ) / vec2(occupied_index_pixel_per_dimension));

    // Use only tile_size - 2*tile_padding pixels to render scene
    // Therefore, scale reduced tile size to full size and translate it
    vec2 padding_scale = 1 - 2 * tile_padding_by_tile_size;
    vec2 padding_offset = tile_padding_by_tile_size;


    // adding the ratio for every texel to our base offset to get the right pixel in our tile
    // and dividing it by the dimension of the phy. tex.
    vec2 physical_texture_coordinates = (base_xy_offset.xy + physical_tile_ratio_xy * padding_scale + padding_offset) / pt.dims;

    // outputting the calculated coordinate from our physical texture
    vec4 c = texture(sampler2DArray(pt.address), vec3(physical_texture_coordinates, index_quadruple.z));

    return c;
}

/*
* Fill the feedback buffer with the feedback value for a given tile.
* Here, we use the maximum lod required from the rendered tile.
*/
void update_feedback(int feedback_value, uvec4 base_offset)
{
    uint one_d_feedback_ssbo_index = base_offset.x + base_offset.y * pt.dims.x + base_offset.z * pt_slice_size;

    int prev = out_lod_feedback_values[one_d_feedback_ssbo_index];
    out_lod_feedback_values[one_d_feedback_ssbo_index] = max(prev, feedback_value);
}

vec4 mix_colors(idx_tex_positions positions, int desired_level, vec2 texture_coordinates, float mix_ratio, int max_level)
{
    vec4 child_color = get_physical_texture_color(positions.child_idx, texture_coordinates, positions.child_lvl, max_level);
    vec4 parent_color = get_physical_texture_color(positions.parent_idx, texture_coordinates, positions.parent_lvl, max_level);

    return true ?
        mix(parent_color, child_color, mix_ratio) : child_color;
}


vec4 traverse_idx_hierarchy(float lambda, vec2 texture_coordinates, usampler2D idx_tex_mm, int max_level)
{
    float mix_ratio = fract(lambda);
    int desired_level = int(ceil(lambda))*3;

    //desired_level = 0;

    idx_tex_positions positions;

    vec4 c = vec4(0.0, 0.0, 0.0, 1.0);
 

    if(desired_level < 0) {
        desired_level = 0;
    }

    {

        //return vec4(0.0, 1.0, 0.0, 0.0);
        // Go from desired tree level downwards to root until a loaded tile is found
        for(int i = desired_level; i >= 0; --i)
        {
            c += vec4(0.1, 0.1, 0.1, 0.0);
            uvec4 idx_child_pos = textureLod(idx_tex_mm, texture_coordinates, max_level-(i) ).rgba;

            // check if the requested tile is loaded and if we are not at the root level
            // enables to mix (linearly interpolate) between hierarchy levels
            if(idx_child_pos.w == 1 && i >= 1)
            {
                uvec4 idx_parent_pos = textureLod(idx_tex_mm, texture_coordinates, max_level-(i-1) ).rgba;
                positions = idx_tex_positions(i-1, idx_parent_pos, i, idx_child_pos);
                break;
            }

            // we are down to the root level: we cannot take the parent node from the root node;
            // therefore, we use the root node as child as well as parent for mixing
            else if(idx_child_pos.w == 1 && i == 0)
            {
                positions = idx_tex_positions(0, idx_child_pos, 0, idx_child_pos);
                break;
            }
        }
    }

    
    {
        c = mix_colors(positions, desired_level, texture_coordinates, mix_ratio, max_level);
    }

    if( int(gl_FragCoord.x) % 128 == 0 && int(gl_FragCoord.y) % 128 == 0 ) {
    	int feedback_value = desired_level;
    	update_feedback(feedback_value, positions.child_idx);
	}

    return c;
}





void main() {

  vec4 uv_lambda_vt_index_quadruple  = texture(gua_uv_buffer, gua_quad_coords).xyzw;
  vec2 sampled_uv_coords = uv_lambda_vt_index_quadruple.xy;
  float lambda 			 = uv_lambda_vt_index_quadruple.z;
  int vt_index = int(round(uv_lambda_vt_index_quadruple.w));

  sampled_uv_coords.y = 1.0 - sampled_uv_coords.y;

  usampler2D index_texture_mip_map_to_sample = usampler2D(vts[vt_index].vt_address);
  int max_level = vts[vt_index].max_level_and_padding.x;
  vec4 virtual_texturing_color = traverse_idx_hierarchy(lambda, sampled_uv_coords, index_texture_mip_map_to_sample, max_level);
  out_vt_color = virtual_texturing_color.rgb;
}
