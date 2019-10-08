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

#ifndef GUA_TEXTURE_HPP
#define GUA_TEXTURE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/math/math.hpp>
#include <gua/utils/Logger.hpp>

// external headers
#include <string>
#include <vector>
#include <mutex>
#include <thread>

#include <boost/uuid/uuid_generators.hpp>
#include <boost/functional/hash.hpp>

namespace gua
{
/**
 * A class representing a texture.
 *
 * This class allows to load texture data from a file and bind the
 * texture to an OpenGL context.
 */
class GUA_DLL Texture
{
  public:
    /**
     * Constructor.
     *
     * This constructs a new texture with the given parameters.
     *
     * \param color_format     The color format of the resulting
     *                         texture.
     * \param state_descripton The sampler state for the loaded texture.
     */
    Texture(scm::gl::data_format color_format,
            scm::gl::data_format internal_format,
            unsigned mipmap_layers = 1,
            scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE));

    /**
     * Constructor.
     *
     * This constructs a new texture with the given parameters.
     *
     * \param color_format     The color format of the resulting
     *                         texture.
     * \param state_descripton The sampler state for the loaded texture.
     */
    Texture(scm::gl::data_format color_format = scm::gl::FORMAT_RGB_32F,
            unsigned mipmap_layers = 1,
            scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_MIP_LINEAR, scm::gl::WRAP_CLAMP_TO_EDGE, scm::gl::WRAP_CLAMP_TO_EDGE));

    /**
     * Constructor.
     *
     * This constructs a new texture from a given file.
     *
     * \param file             The file which contains the texture data.
     * \param state_descripton The sampler state for the loaded texture.
     */
    Texture(std::string const& file,
            bool generate_mipmaps = false,
            scm::gl::sampler_state_desc const& state_descripton = scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_REPEAT, scm::gl::WRAP_REPEAT));

    virtual ~Texture() {}

    void update_sub_data(RenderContext const& context, scm::gl::texture_region const& region, unsigned level, scm::gl::data_format format, const void* const data) const;

    void generate_mipmaps(RenderContext const& context);

    /**
     *
     */
    virtual math::vec2ui const get_handle(RenderContext const& context) const;

    /**
     * Get the schism texture.
     *
     * \param context          The context for which the texture should be
     *                         returned.
     * \return                 A pointer to the schism texture.
     */
    scm::gl::texture_image_ptr const& get_buffer(RenderContext const& context) const;

    void make_non_resident(RenderContext const& context) const;

    virtual unsigned width() const = 0;
    virtual unsigned height() const = 0;

    virtual void upload_to(RenderContext const& context) const = 0;

    std::size_t uuid() { return uuid_; }

  protected:
    unsigned mipmap_layers_;
    scm::gl::data_format color_format_;
    scm::gl::data_format internal_format_;
    scm::gl::sampler_state_desc state_descripton_;
    std::string file_name_;

    mutable std::mutex upload_mutex_;

    std::size_t uuid_ = boost::hash<boost::uuids::uuid>()(boost::uuids::random_generator()());
};

} // namespace gua
#endif // GUA_TEXTURE2D_HPP
