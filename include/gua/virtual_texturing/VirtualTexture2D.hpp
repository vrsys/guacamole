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

#ifndef GUA_VIRTUALTEXTURE2D_HPP
#define GUA_VIRTUALTEXTURE2D_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/virtual_texturing/LayeredPhysicalTexture2D.hpp>

//#include <lamure/vt/VTConfig.h>

#include <scm/gl_util/data/imaging/texture_image_data.h>

// external headers
#include <string>
#include <vector>

#include <mutex>
#include <thread>
#include <memory>

namespace gua {

class GUA_DLL VirtualTexture2D : public Texture {
 public:
  
  VirtualTexture2D(std::string const& file,
                   std::size_t physical_texture_tile_slot_size,
                   scm::gl::sampler_state_desc const& state_descripton =
                   scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_NEAREST,
                                               scm::gl::WRAP_REPEAT,
                                               scm::gl::WRAP_REPEAT));

  unsigned width() const override { return _physical_texture_width; }
  unsigned height() const override { return _physical_texture_height; }
  unsigned get_physical_texture_width() const { return _physical_texture_width; }
  unsigned get_physical_texture_height() const { return _physical_texture_height; }
  uint32_t get_tile_size() const { return _tile_size; }
  uint32_t get_lamure_texture_id() const { return _lamure_texture_id; }

  uint32_t get_max_depth() const {return max_depth_;}

  void upload_to(RenderContext const& context) const override {};
  void upload_to(RenderContext const& context, uint32_t num_hierarchy_levels) const;

  void initialize_index_texture(RenderContext const& ctx, uint64_t cut_id) const;

  // per render (gua) contexts
  static std::map<std::size_t,
            std::shared_ptr<LayeredPhysicalTexture2D> > physical_texture_ptr_per_context_;

  // per render (gua) contexts
  static std::map<std::size_t, VTInfo> vt_info_per_context_;

  //static scm::math::vec2ui get_physical_texture_handle(RenderContext const& ctx) {
  //  return physical_texture_ptr_per_context_[ctx.id]->get_physical_texture_handle(ctx);
  //}

  //std::vector<scm::gl::texture_2d_ptr>& get_index_texture_ptrs_for_context(RenderContext const& ctx) {
  //  return index_texture_hierarchy_per_context_[ctx.id];
  //}

  scm::gl::texture_2d_ptr& get_index_texture_ptrs_for_context(RenderContext const& ctx) {
    return index_texture_mip_map_per_context_[ctx.id];
  }

 protected:

  //mutable std::map<std::size_t,
  //  std::vector<scm::gl::texture_2d_ptr> > index_texture_hierarchy_per_context_;
  
  mutable std::map<std::size_t,
    scm::gl::texture_2d_ptr>               index_texture_mip_map_per_context_;
  //scm::gl::texture_image_data_ptr image_ = nullptr;
  unsigned _physical_texture_width;
  unsigned _physical_texture_height;
  uint16_t _tile_size;

  mutable uint32_t max_depth_;

  //unsigned layers_;

 private:
  std::string _file_config;
  std::string _file_atlas;
  uint32_t    _lamure_texture_id;
  scm::shared_ptr<scm::gl::render_device> _device;
  mutable scm::math::vec2ui                                                         _index_texture_dimension;
  scm::gl::sampler_state_ptr                                                        _filter_nearest;
  scm::gl::sampler_state_ptr                                                        _filter_linear;
};  

}
#endif  // GUA_VIRTUALTEXTURE2D_HPP
