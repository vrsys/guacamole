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
namespace gua {

//std::map<std::size_t, std::vector<std::ifstream>>         TV_3ResourceVQCompressed::per_resource_codebook_file_streams_;
//std::map<std::size_t, std::vector<std::vector<uint8_t >>> TV_3ResourceVQCompressed::per_resource_codebook_cpu_cache_;

////////////////////////////////////////////////////////////////////////////////

TV_3ResourceVQCompressed::TV_3ResourceVQCompressed(std::string const& resource_file_string, bool is_pickable)
  : TV_3Resource(resource_file_string, is_pickable) {
/*
      std::cout << "Loading once\n";
  bounding_box_.min = scm::math::vec3(0.0f, 0.0f, 0.0f);
  bounding_box_.max = scm::math::vec3(1.0f, 1.0f, 1.0f);

  std::vector<std::string> volumes_to_load;
  if(resource_file_name_.find(".v_rsc") != std::string::npos) {
    std::string line_buffer;
    std::ifstream volume_resource_file(resource_file_name_, std::ios::in);
    while(std::getline(volume_resource_file, line_buffer)) {
      if(line_buffer.find(".raw") != std::string::npos) {
        volumes_to_load.push_back(line_buffer);
      }
    }
  } else {
    volumes_to_load.push_back(resource_file_name_);
  }

  uint8_t num_parsed_volumes = 0;
  for( auto const& vol_path : volumes_to_load ) {
    if( 0 == num_parsed_volumes++ ) {
      tokenize_volume_name(vol_path, volume_descriptor_tokens_[uuid_]);  
    }
    
    per_resource_file_streams_[uuid_].push_back(std::ifstream(vol_path.c_str(), std::ios::in | std::ios::binary));

  }
    */
}

////////////////////////////////////////////////////////////////////////////////

TV_3ResourceVQCompressed::~TV_3ResourceVQCompressed() {
}



void TV_3ResourceVQCompressed::upload_to(RenderContext const& ctx) const {
  TV_3Resource::upload_to(ctx);
  /*
  int64_t loaded_volumes_count = 0;

  auto& current_tokens = volume_descriptor_tokens_[uuid_];

  for( auto& vol_path : per_resource_file_streams_[uuid_] ) { 
    scm::math::vec3ui vol_dims = scm::math::vec3ui(current_tokens["w"], current_tokens["h"], current_tokens["d"]);

    int64_t num_bytes_per_voxel = current_tokens["num_bytes_per_voxel"];

    int64_t num_voxels = current_tokens["total_num_bytes"];
    //std::vector<unsigned char> read_buffer(num_voxels);

    per_resource_cpu_cache_[uuid_].push_back(std::vector<uint8_t>(num_voxels, 0));
    vol_path.read( (char*) &per_resource_cpu_cache_[uuid_][loaded_volumes_count][0], num_voxels);


    scm::gl::data_format read_format = scm::gl::data_format::FORMAT_NULL;

    if( 1 == num_bytes_per_voxel ) {
      read_format = scm::gl::data_format::FORMAT_R_8;
    } else if( 2 == num_bytes_per_voxel ) {
      read_format = scm::gl::data_format::FORMAT_R_16;   
    } else if( 4 == num_bytes_per_voxel ) {
      read_format = scm::gl::data_format::FORMAT_R_32F;   
    }

    volume_textures_.push_back(ctx.render_device->create_texture_3d(scm::gl::texture_3d_desc(vol_dims, read_format), read_format, 
                                                                    {(void*) &per_resource_cpu_cache_[uuid_][loaded_volumes_count][0]} ) );
    ++loaded_volumes_count;
  }


  std::make_shared<scm::gl::texture_3d>(
                                                          new scm::gl::texture_3d(*ctx.render_device, 
                                                              scm::gl::texture_3d_desc(scm::math::vec3ui(256,256,225),
                                                                                  scm::gl::data_format::FORMAT_R_8)  );
*/
}

void TV_3ResourceVQCompressed::bind_volume_texture(
  RenderContext const& ctx, scm::gl::sampler_state_ptr const& sampler_state) const {

  //TV_3Resource::bind_volume_texture(ctx, sampler_state);

  if( volume_textures_.empty() ) {
    upload_to(ctx);
  }

  ctx.render_context->bind_texture(volume_textures_[ ((frame_counter_++) / 10) % volume_textures_.size()], sampler_state, 0);

/*
  std::cout << "In drawing branch\n";
  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->apply();
  std::cout << "Before drawing\n";
  volume_proxy_.draw(ctx.render_context);
*/
}





}
