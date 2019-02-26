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

#ifndef GUA_TEXTURE_DATABASE_HPP
#define GUA_TEXTURE_DATABASE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>
#include <gua/renderer/Texture.hpp>

#include <mutex>
#include <future>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
#include <gua/virtual_texturing/VirtualTexture2D.hpp>
#endif

namespace gua
{
/**
 * A data base for textures.
 *
 * This Database stores texture data. It can be accessed via string
 * identifiers.
 *
 * \ingroup gua_databases
 */
class GUA_DLL TextureDatabase : public Database<Texture>, public Singleton<TextureDatabase>
{
  public:
    /**
     * Loads a texture file to the database.
     *
     * This method loads textures to the data base.
     *
     * \param id  An absolute or relative path to the
     *            directory containing texture files.
     */
    void load(std::string const& id);

    int32_t get_global_texture_id_by_path(std::string const& tex_path) const;

    friend class Singleton<TextureDatabase>;

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    std::vector<std::shared_ptr<VirtualTexture2D>> get_virtual_textures();
#endif

  private:
    // this class is a Singleton --- private c'tor and d'tor
    TextureDatabase();
    ~TextureDatabase() = default;

    std::vector<std::future<std::string>> textures_loading_;
    std::mutex texture_request_mutex_;
    std::set<std::string> texture_loading_;

    std::unordered_map<std::string, uint32_t> texture_path_to_global_id_mapping_;
    uint32_t num_loaded_textured_ = 0;

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
    std::unordered_map<std::string, std::shared_ptr<VirtualTexture2D>> virtual_textures_;
#endif
};

} // namespace gua

#endif // GUA_TEXTURE_DATABASE_HPP
