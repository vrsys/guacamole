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

#ifndef GUA_LAYEREDPHYSICALTEXTURE2D_HPP
#define GUA_LAYEREDPHYSICALTEXTURE2D_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/Texture.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/Logger.hpp>

#include <gua/virtual_texturing/VTInfo.hpp>

#include <scm/gl_util/data/imaging/texture_image_data.h>

// external headers
#include <string>
#include <vector>

#include <mutex>
#include <thread>
#include <memory>

namespace gua
{
/**
 * A class representing a texture.
 *
 * This class allows to load texture data from a file and bind the
 * texture to an OpenGL context.
 */
class GUA_DLL LayeredPhysicalTexture2D
{ // : public Texture {
  public:
    /**
     * Constructor.
     *
     * This constructs a new texture from a given file.
     *
     * \param file             The file which contains the texture data.
     * \param state_descripton The sampler state for the loaded texture.
     */
    LayeredPhysicalTexture2D(); /*std::string const& file,
                scm::gl::sampler_state_desc const& state_descripton =
                scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC,
                                            scm::gl::WRAP_REPEAT,
                                            scm::gl::WRAP_REPEAT));*/

    ~LayeredPhysicalTexture2D();

    // Constructor
    // get path to config file and atlas
    // retrieve size from backend
    // VTTexture2D(gua::math::vec2ui dim, uint16_t layers, vt::VTConfig::FORMAT_TEXTURE format);

    ///@{
    /**
     * Gets the size.
     *
     * Returns the size of the Texture2D.
     */
    unsigned width() const { return width_; }
    unsigned height() const { return height_; }
    unsigned num_layers() const { return num_layers_; }
    ///@}

    void upload_to(RenderContext const& context) const {};
    void upload_to(RenderContext const& ctx, uint32_t width, uint32_t height, uint32_t num_layers, uint32_t tile_size) const;

    void upload_physical_texture_handle_to_ubo(RenderContext const& ctx) const;

    scm::gl::texture_2d_ptr get_physical_texture_ptr() const { return physical_texture_ptr_; }
    scm::gl::buffer_ptr get_feedback_lod_storage_ptr() const { return feedback_lod_storage_; }
#ifdef RASTERIZATION_COUNT
    scm::gl::buffer_ptr get_feedback_count_storage_ptr() const { return feedback_count_storage_; }
#endif
    // static scm::gl::data_format get_tex_format();
    scm::gl::buffer_ptr& get_feedback_index_ib() const { return feedback_index_ib_; }
    scm::gl::buffer_ptr& get_feedback_index_vb() const { return feedback_index_vb_; }
    scm::gl::vertex_array_ptr& get_feedback_vao() const { return feedback_vao_; }
    scm::gl::buffer_ptr& get_feedback_inv_index() const { return feedback_inv_index_; }

    int32_t* get_feedback_lod_cpu_buffer() const { return feedback_lod_cpu_buffer_; }
#ifdef RASTERIZATION_COUNT
    uint32_t* get_feedback_count_cpu_buffer() const { return feedback_count_cpu_buffer_; }
#endif

    std::size_t get_num_feedback_slots() const { return num_feedback_slots_; }

  protected:
    mutable scm::gl::texture_2d_ptr physical_texture_ptr_ = nullptr;

    mutable scm::gl::buffer_ptr feedback_index_ib_ = nullptr;
    mutable scm::gl::buffer_ptr feedback_index_vb_ = nullptr;
    mutable scm::gl::vertex_array_ptr feedback_vao_ = nullptr;
    mutable scm::gl::buffer_ptr feedback_inv_index_ = nullptr;
    mutable scm::gl::buffer_ptr feedback_lod_storage_ = nullptr;
#ifdef RASTERIZATION_COUNT
    mutable scm::gl::buffer_ptr feedback_count_storage_ = nullptr;
#endif

    mutable scm::gl::buffer_ptr physical_texture_address_ubo_ = nullptr;
    mutable scm::gl::buffer_ptr vt_addresses_ubo_per_context_ = nullptr;

    mutable int32_t* feedback_lod_cpu_buffer_ = nullptr;
#ifdef RASTERIZATION_COUNT
    mutable uint32_t* feedback_count_cpu_buffer_ = nullptr;
#endif

    mutable unsigned width_;
    mutable unsigned height_;
    mutable unsigned num_layers_;
    mutable unsigned tile_size_;

    mutable std::size_t num_feedback_slots_;

  private:
    std::string file_config_;
    std::string file_atlas_;
};

} // namespace gua
#endif // GUA_LAYEREDPHYSICALTEXTURE2D_HPP