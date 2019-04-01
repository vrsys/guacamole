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
#include <gua/databases/TextureDatabase.hpp>
#include <gua/renderer/Texture2D.hpp>
#include <gua/renderer/Texture3D.hpp>

// guacamole headers
#include <gua/utils/Directory.hpp>

// external headers
#include <sstream>
#include <future>
#include <iostream>
#include <cstdint>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <gua/renderer/Texture2D.hpp>

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
#include <gua/virtual_texturing/VirtualTexture2D.hpp>
#include <gua/virtual_texturing/LayeredPhysicalTexture2D.hpp>
#endif

namespace gua
{
TextureDatabase::TextureDatabase()
{
    texture_path_to_global_id_mapping_["gua_loading_texture"] = 0;
    texture_path_to_global_id_mapping_["gua_default_texture"] = 1;
    texture_path_to_global_id_mapping_["gua_noise_texture"] = 2;
    num_loaded_textured_ = 3;
}

void TextureDatabase::load(std::string const& filename)
{
    boost::filesystem::path fp(filename);
    std::string extension(fp.extension().string());
    boost::algorithm::to_lower(extension);

    if(extension == ".png" || extension == ".jpg" || extension == ".jpeg" || extension == ".bmp" || extension == ".dds" || extension == ".tif" || extension == ".tga")
    {
        {
            std::lock_guard<std::mutex> lock(texture_request_mutex_);
            auto needs_to_be_loaded_it = texture_loading_.find(filename);

            if(texture_loading_.end() != needs_to_be_loaded_it)
                return;

            texture_loading_.insert(filename);
        }

        // else
        textures_loading_.push_back(std::async(std::launch::async, [filename]() -> std::string {
            auto default_tex = TextureDatabase::instance()->lookup("gua_default_texture");
            if(default_tex)
            {
                gua::TextureDatabase::instance()->add(filename, default_tex);
            }

            auto image = gua::load_image_2d(filename, true);

            instance()->add(filename, std::make_shared<Texture2D>(image, 1, scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC, scm::gl::WRAP_REPEAT, scm::gl::WRAP_REPEAT)));
            return filename;
        }));
    }
    else if(extension == ".vol")
    {
        instance()->add(filename, std::make_shared<Texture3D>(filename, true));
    }
    else if(".atlas" == extension)
    {
#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
        auto occurrence_check = TextureDatabase::instance()->lookup(filename);

        std::shared_ptr<VirtualTexture2D> vt_pointer = nullptr;
        if(!occurrence_check)
        {
            vt_pointer = std::make_shared<VirtualTexture2D>(filename, scm::gl::sampler_state_desc(scm::gl::FILTER_MIN_MAG_NEAREST, scm::gl::WRAP_REPEAT, scm::gl::WRAP_REPEAT));
            instance()->add(filename, vt_pointer);
        }

        auto existing_vt = TextureDatabase::instance()->lookup(filename);

        if(!existing_vt)
        {
            std::cout << "Failed to create Virtual Texture: " << filename << "\n";
        }
        else
        {
            virtual_textures_[filename] = vt_pointer;
        }

#else
        Logger::LOG_ERROR << "Unable to load .atlas-Texture: Virtual Texturing plugin is not enabled!" << std::endl;
#endif
    }
    else
    {
        // Logger::LOG_ERROR << "Unable to load texture: \"" << filename <<"\": Unknown File Format.";
        return;
    }

    auto texture_it = texture_path_to_global_id_mapping_.find(filename);
    if(texture_path_to_global_id_mapping_.end() == texture_it)
    {
        texture_path_to_global_id_mapping_[filename] = num_loaded_textured_++;
    }
}

int32_t TextureDatabase::get_global_texture_id_by_path(std::string const& tex_path) const
{
    auto texture_it = texture_path_to_global_id_mapping_.find(tex_path);
    if(texture_path_to_global_id_mapping_.end() != texture_it)
    {
        return texture_it->second;
    }

    // Logger::LOG_ERROR << "Texture ID for \"" << tex_path << "\" was not registered." << std::endl;
    return -1;
}

#ifdef GUACAMOLE_ENABLE_VIRTUAL_TEXTURING
std::vector<std::shared_ptr<VirtualTexture2D>> TextureDatabase::get_virtual_textures()
{
    std::vector<std::shared_ptr<VirtualTexture2D>> virtual_texture_ptrs;

    for(auto const& vt : virtual_textures_)
    {
        virtual_texture_ptrs.push_back(vt.second);
    }

    return virtual_texture_ptrs;
}
#endif

} // namespace gua