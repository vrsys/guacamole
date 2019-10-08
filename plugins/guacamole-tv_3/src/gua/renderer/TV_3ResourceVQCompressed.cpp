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
#include <gua/renderer/TV_3ResourceVQCompressed.hpp>

#include <fstream>
/*
#include <gua/utils/Singleton.hpp>
#include <gua/node/TV_3Node.hpp>

#include <scm/gl_core/render_device.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/texture_objects.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>

#include <scm/gl_util/data/volume/volume_loader.h>

#include <boost/assign/list_of.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/node/Node.hpp>

//#include <pbr/ren/ray.h>
//#include <pbr/ren/controller.h>

// external headers
#include <stack>
#include <algorithm>
#include <regex>
*/
namespace gua
{
std::mutex TV_3ResourceVQCompressed::cpu_codebook_loading_mutex_;
std::map<std::size_t, bool> TV_3ResourceVQCompressed::are_cpu_codebooks_loaded_;
std::map<std::size_t, std::vector<std::ifstream>> TV_3ResourceVQCompressed::per_resource_codebook_file_streams_;
std::map<std::size_t, std::vector<std::vector<uint8_t>>> TV_3ResourceVQCompressed::per_resource_codebook_cpu_cache_;

////////////////////////////////////////////////////////////////////////////////

TV_3ResourceVQCompressed::TV_3ResourceVQCompressed(std::string const& resource_file_string, bool is_pickable) : TV_3Resource(resource_file_string, is_pickable, CompressionMode::SW_VQ)
{
    std::cout << "Created Compressed Volume Resource\n";

    bool is_multi_codebook_mode = false;

    cpu_codebook_loading_mutex_.lock();

    if(!are_cpu_codebooks_loaded_[uuid_])
    {
        std::vector<std::string> volumes_to_load;
        if(resource_file_name_.find(".v_rsc") != std::string::npos)
        {
            if(resource_file_name_.find("MCM") != std::string::npos)
            {
                is_multi_codebook_mode = true;
            }

            std::string line_buffer;
            std::ifstream volume_resource_file(resource_file_name_, std::ios::in);
            while(std::getline(volume_resource_file, line_buffer))
            {
                if(line_buffer.find(".raw") != std::string::npos)
                {
                    volumes_to_load.push_back(line_buffer + ".cb");
                    std::cout << "Pushing back: " + line_buffer + ".cb\n\n";

                    if(!is_multi_codebook_mode)
                    {
                        break;
                    }
                }
            }
        }
        else
        {
            volumes_to_load.push_back(resource_file_name_);
        }

        uint8_t num_parsed_volumes = 0;
        for(auto const& vol_path : volumes_to_load)
        {
            if(0 == num_parsed_volumes++)
            {
                tokenize_volume_name(vol_path, volume_descriptor_tokens_[uuid_]);
            }

            per_resource_codebook_file_streams_[uuid_].push_back(std::ifstream(vol_path.c_str(), std::ios::in | std::ios::binary | std::ios::ate));
        }

        are_cpu_codebooks_loaded_[uuid_] = true;
    }

    cpu_codebook_loading_mutex_.unlock();
}

////////////////////////////////////////////////////////////////////////////////

TV_3ResourceVQCompressed::~TV_3ResourceVQCompressed() {}

void TV_3ResourceVQCompressed::upload_to(RenderContext const& ctx) const
{
    TV_3Resource::upload_to(ctx);

    if(are_cpu_codebooks_loaded_[uuid_])
    {
        int64_t loaded_volumes_count = 0;

        auto& current_tokens = volume_descriptor_tokens_[uuid_];
        int64_t num_bytes_per_voxel = current_tokens["num_bytes_per_voxel"];

        int64_t total_block_size = std::pow(current_tokens["bs"], 3);
        int64_t size_per_codeword = (num_bytes_per_voxel)*total_block_size;

        int64_t const MAX_CODEBOOK_WIDTH = 16384;

        current_tokens["num_codewords_per_row"] = std::floor(MAX_CODEBOOK_WIDTH / total_block_size);
        int64_t codebook_width = current_tokens["num_codewords_per_row"] * total_block_size;

        size_t size_of_data_type = current_tokens["b"];

        for(auto& codebook_stream : per_resource_codebook_file_streams_[uuid_])
        {
            int64_t actual_num_index_bit_power = current_tokens["a"];
            int64_t num_codewords = std::pow(2, actual_num_index_bit_power);
            int64_t codebook_height = int64_t(std::ceil(num_codewords / (float)current_tokens["num_codewords_per_row"]));

            int64_t codebook_texture_num_bytes = codebook_height * codebook_width * num_bytes_per_voxel;
            int64_t num_bytes_in_codebook_file = codebook_stream.tellg();
            codebook_stream.seekg(0);
            std::cout << "Reading num bytes: " << num_bytes_in_codebook_file << "\n";

            per_resource_codebook_cpu_cache_[uuid_].push_back(std::vector<uint8_t>(codebook_texture_num_bytes, 0));
            codebook_stream.read((char*)&(per_resource_codebook_cpu_cache_[uuid_][loaded_volumes_count][0]), num_bytes_in_codebook_file);

            current_tokens["codebook_width"] = codebook_width;
            current_tokens["codebook_height"] = codebook_height;

            scm::math::vec2ui codebook_dims = scm::math::vec2ui(current_tokens["codebook_width"], current_tokens["codebook_height"]);

            scm::gl::data_format read_format = scm::gl::data_format::FORMAT_NULL;

            if(1 == num_bytes_per_voxel)
            {
                read_format = scm::gl::data_format::FORMAT_R_8;
            }
            else if(2 == num_bytes_per_voxel)
            {
                read_format = scm::gl::data_format::FORMAT_R_16;
            }
            else if(4 == num_bytes_per_voxel)
            {
                read_format = scm::gl::data_format::FORMAT_R_32F;
                std::cout << "Reading 32 bit codewords\n";
            }

            std::cout << "CONTENT: \n";
            /*
                for( auto& element : per_resource_codebook_cpu_cache_[uuid_][loaded_volumes_count] ) {
                  std::cout << element << "\n";
                }*/

            ctx.texture_2d_arrays[uuid()].push_back(
                ctx.render_device->create_texture_2d(scm::gl::texture_2d_desc(codebook_dims, read_format), read_format, {(void*)&(per_resource_codebook_cpu_cache_[uuid_][loaded_volumes_count][0])}));

            ++loaded_volumes_count;
        }

        num_codebooks_ = loaded_volumes_count;
    }
}

void TV_3ResourceVQCompressed::apply_resource_dependent_uniforms(RenderContext const& ctx, std::shared_ptr<ShaderProgram> const& current_program) const
{
    TV_3Resource::apply_resource_dependent_uniforms(ctx, current_program);

    auto& current_tokens = volume_descriptor_tokens_[uuid_];
    current_program->apply_uniform(ctx, "num_codewords_per_row", int32_t(current_tokens["num_codewords_per_row"]));
    current_program->apply_uniform(ctx, "block_offset_vector", math::vec3i(1, current_tokens["bs"], current_tokens["bs"] * current_tokens["bs"]));
    current_program->apply_uniform(ctx, "total_block_size", int32_t(current_tokens["bs"] * current_tokens["bs"] * current_tokens["bs"]));
    current_program->apply_uniform(ctx, "volume_dimensions", math::vec3i(current_tokens["w"], current_tokens["h"], current_tokens["d"]));
    // current_program->apply_uniform(ctx, "total_block_size", )
    /*
      uniform ivec3 block_offset_vector = ivec3(0, 0, 0);
    uniform int total_block_size = 0;
    uniform ivec3 volume_dimensions = ivec3(0, 0, 0);*/
};

void TV_3ResourceVQCompressed::bind_volume_texture(RenderContext const& ctx, scm::gl::sampler_state_ptr const& sampler_state) const
{
    TV_3Resource::bind_volume_texture(ctx, sampler_state);

    /*
      if( volume_textures_.empty() && codebook_textures_.empty() ) {
        upload_to(ctx);
      }
    */
    auto nearest_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);

    int32_t codebook_id = int32_t(time_cursor_pos_) % num_codebooks_;

    ctx.render_context->bind_texture(ctx.texture_2d_arrays[uuid()][codebook_id], nearest_sampler_state_, 1);

    ctx.render_context->apply_texture_units();
    // the minus 1 hack currently only applies because the base class increments the counter. The hack is removed during on of the next iterations
}

} // namespace gua
