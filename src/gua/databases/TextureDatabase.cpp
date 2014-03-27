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
#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

namespace gua {

void TextureDatabase::load(std::string const& id) {
  boost::filesystem::path fp(id);
  std::string extension(fp.extension().string());
  boost::algorithm::to_lower(extension);

  if (extension == ".png" || extension == ".jpg" || extension == ".bmp" ||
      extension == ".tif" || extension == ".tga") {
    instance()->add(id, std::make_shared<Texture2D>(id, true));
  } else if (extension == ".vol") {
    instance()->add(id, std::make_shared<Texture3D>(id, true));
  }
}

}
