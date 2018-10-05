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
#include <gua/virtual_texturing/VirtualTexture2D.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/math.hpp>

// external headers
#include <boost/shared_ptr.hpp>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/log.h>
#include <scm/gl_core/render_device.h>
#include <scm/gl_core/texture_objects.h>

#include <scm/gl_util/data/imaging/texture_loader.h>
#include <scm/gl_util/data/imaging/texture_image_data.h>

#include <lamure/vt/VTConfig.h>
#include <lamure/vt/ren/CutDatabase.h>
#include <lamure/vt/pre/AtlasFile.h>

#include <iostream>
#include <fstream>
#include <regex>

#define PHYSICAL_TEXTURE_MAX_NUM_LAYERS 256
#define PHYSICAL_TEXTURE_MAX_RES_PER_AXIS 8192

#define MAX_VIRTUAL_TEXTURES 32

namespace gua {
  std::map<std::size_t,
    std::shared_ptr<LayeredPhysicalTexture2D> > VirtualTexture2D::physical_texture_ptr_per_context_ 
      = std::map<std::size_t, std::shared_ptr<LayeredPhysicalTexture2D> >();

  std::map<std::size_t, VTInfo> VirtualTexture2D::vt_info_per_context_ 
      = std::map<std::size_t, VTInfo>();

  VirtualTexture2D::VirtualTexture2D(std::string const& atlas_filename,
                                     std::size_t physical_texture_tile_slot_size,
                                     scm::gl::sampler_state_desc const& state_descripton) {

    std::string const ini_filename = std::regex_replace(atlas_filename, std::regex(".atlas"), ".ini");

    ::vt::VTConfig::CONFIG_PATH = ini_filename;
    ::vt::VTConfig::get_instance().define_size_physical_texture(PHYSICAL_TEXTURE_MAX_NUM_LAYERS, PHYSICAL_TEXTURE_MAX_RES_PER_AXIS);
    _tile_size = ::vt::VTConfig::get_instance().get_size_tile();

    _lamure_texture_id = ::vt::CutDatabase::get_instance().register_dataset(atlas_filename);

    ::vt::pre::AtlasFile current_atlas_file(atlas_filename.c_str());

    max_depth_ = current_atlas_file.getDepth();

    //std::cout << "MAX DEPTH AS DEFINED BY ATLAS FILE: " << max_depth_ << "\n";
/*

    std::cout << "Defined physical_texture_siye\n";


    std::size_t tile_size = 0;
    std::string line_buffer{""};

    std::ifstream ini_filestream(ini_filename, std::ios::in);

    while(std::getline(ini_filestream, line_buffer)) {
      trim(line_buffer);
      if(0 == line_buffer.find("TILE_SIZE=")) {
        tile_size = std::stoi(line_buffer.substr(10));
      }
    }
    ini_filestream.close();

    if(physical_texture_tile_slot_size == tile_size) {
      std::cout << "Tile size compatible\n";
    } else {
      std::cout << "Tile size Incompatible\n";
    }
*/
  }



  void VirtualTexture2D::upload_to(RenderContext const& ctx) const {

    auto index_texture_hierarchy_context_iterator = index_texture_mip_map_per_context_.find(ctx.id);

    if(nullptr == nearest_mip_map_sampler_state_) {
      nearest_mip_map_sampler_state_ = ctx.render_device->create_sampler_state(scm::gl::FILTER_MIN_MAG_MIP_NEAREST, scm::gl::WRAP_CLAMP_TO_EDGE);
    }

    if(index_texture_hierarchy_context_iterator == index_texture_mip_map_per_context_.end()) {
      //max_depth_ = num_hierarchy_levels + 1;

        uint32_t size_index_texture = (uint32_t) vt::QuadTree::get_tiles_per_row(max_depth_);

        auto index_texture_level_ptr = ctx.render_device->create_texture_2d(
                                                                            scm::math::vec2ui(size_index_texture, size_index_texture), 
                                                                            scm::gl::FORMAT_RGBA_8UI,
                                                                            max_depth_ + 1);

        for(uint32_t i = 0; i < max_depth_ + 1; ++i) {
          ctx.render_context->clear_image_data(index_texture_level_ptr, i, scm::gl::FORMAT_RGBA_8UI, 0);
        }

        index_texture_mip_map_per_context_[ctx.id] = index_texture_level_ptr;   

        ctx.render_context->make_resident(index_texture_level_ptr, nearest_mip_map_sampler_state_);

        upload_vt_handle_to_ubo(ctx);
    }
  }

  void VirtualTexture2D::upload_vt_handle_to_ubo(RenderContext const& ctx) const {

    if(vt_addresses_ubo_per_context_.end() == vt_addresses_ubo_per_context_.find(ctx.id) ) {
      vt_addresses_ubo_per_context_[ctx.id] = ctx.render_device->create_buffer(scm::gl::BIND_UNIFORM_BUFFER, scm::gl::USAGE_STATIC_DRAW,
                                                                               MAX_VIRTUAL_TEXTURES * sizeof(scm::math::vec2ui));
    }

    auto& current_vt_addresses_ubo = vt_addresses_ubo_per_context_[ctx.id];



    uint64_t handle = index_texture_mip_map_per_context_[ctx.id]->native_handle();

    uint64_t physical_texture_cpu_address = (handle & 0x00000000ffffffff) | (handle & 0xffffffff00000000);
    //math::vec2ui swapped_texture_adress = math::vec2ui(handle & 0x00000000ffffffff, handle & 0xffffffff00000000);

    uint64_t *mapped_physical_texture_address_ubo = (uint64_t *) ctx.render_context->map_buffer(current_vt_addresses_ubo,
                                                                                                scm::gl::ACCESS_WRITE_ONLY);
    memcpy(&mapped_physical_texture_address_ubo[0], &physical_texture_cpu_address, sizeof(uint64_t));
    ctx.render_context->unmap_buffer(current_vt_addresses_ubo);


    ctx.render_context->bind_uniform_buffer(current_vt_addresses_ubo, 3);
  }


  void VirtualTexture2D::initialize_index_texture(RenderContext const& ctx, uint64_t cut_id) const {

  }
}
