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

namespace gua {

void TextureDatabase::load(std::string const& filename) {
  boost::filesystem::path fp(filename);
  std::string extension(fp.extension().string());
  boost::algorithm::to_lower(extension);

  if (extension == ".png"
      || extension == ".jpg"
      || extension == ".jpeg"
      || extension == ".bmp"
      || extension == ".dds"
      || extension == ".tif"
      || extension == ".tga") {

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
        if (default_tex) {
          gua::TextureDatabase::instance()->add(filename, default_tex);
        }

        auto image = gua::load_image_2d(filename, true);

        instance()->add(filename, std::make_shared<Texture2D>(image, 1,
              scm::gl::sampler_state_desc(scm::gl::FILTER_ANISOTROPIC,
                                          scm::gl::WRAP_REPEAT,
                                          scm::gl::WRAP_REPEAT)));
        return filename;
      }));


  } else if (extension == ".vol") {
    instance()->add(filename, std::make_shared<Texture3D>(filename, true));
  }
}

}
