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
#include <gua/math/math.hpp>
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/virtual_texturing/LayeredPhysicalTexture2D.hpp>

//#include <lamure/vt/VTConfig.h>

#include <scm/gl_util/data/imaging/texture_image_data.h>

// external headers
#include <string>
#include <vector>

#include <memory>
#include <mutex>
#include <thread>

namespace gua
{
class GUA_DLL VirtualTexture2D : public Texture
{
  public:
    VirtualTexture2D(std::string const& file,
                     std::size_t physical_texture_tile_slot_size,
                     scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_REPEAT, scm::gl::WRAP_REPEAT));

    unsigned width() const override { return 0; }
    unsigned height() const override { return 0; }
    uint32_t get_tile_size() const { return tile_size_; }
    uint32_t get_lamure_texture_id() const { return lamure_texture_id_; }

    uint32_t get_max_depth() const { return max_depth_; }

    void upload_to(RenderContext const& context) const override;

    // static scm::math::vec2ui get_physical_texture_handle(RenderContext const& ctx) {
    //  return physical_texture_ptr_per_context_[ctx.id]->get_physical_texture_handle(ctx);
    //}

    // std::vector<scm::gl::texture_2d_ptr>& get_index_texture_ptrs_for_context(RenderContext const& ctx) {
    //  return index_texture_hierarchy_per_context_[ctx.id];
    //}

    void upload_vt_handle_to_ubo(RenderContext const& ctx) const;

    // scm::gl::texture_2d_ptr& get_index_texture_ptrs_for_context(RenderContext const& ctx) {
    //  return index_texture_mip_map_per_context_[ctx.id];
    //}

    void update_index_texture_hierarchy(RenderContext const& ctx, std::vector<std::pair<uint16_t, uint8_t*>> const& level_update_pairs);

  protected:
    // mutable std::map<std::size_t,
    //  std::vector<scm::gl::texture_2d_ptr> > index_texture_hierarchy_per_context_;

    mutable std::map<std::size_t, scm::gl::texture_2d_ptr> index_texture_mip_map_per_context_;

    static std::map<std::size_t, scm::gl::buffer_ptr> vt_addresses_ubo_per_context_;

    // scm::gl::texture_image_data_ptr image_ = nullptr;
    uint16_t tile_size_;

    mutable uint32_t max_depth_;
    // unsigned layers_;

  private:
    std::string ini_file_path_;
    std::string atlas_file_path_;
    uint32_t lamure_texture_id_;

    static bool initialized_vt_system;
};
} // namespace gua
#endif // GUA_VIRTUALTEXTURE2D_HPP
