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

#ifndef GUA_NOISE_TEXTURE_HPP
#define GUA_NOISE_TEXTURE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/Texture2D.hpp>

namespace gua {

class GUA_DLL NoiseTexture : public Texture2D {
 public:

  NoiseTexture();

 private:
  static unsigned char pixel_data[64 * 64 * 3 + 1];
};

class GUA_DLL DefaultTexture : public Texture2D {
 public:

  DefaultTexture();

 private:

#if WIN32
  static unsigned char pixel_data[64 * 64 * 3 + 1];
#else
  static unsigned char pixel_data[256 * 256 * 3 + 1];
#endif
};

class GUA_DLL LoadingTexture : public Texture2D {
 public:

  LoadingTexture();

 private:
  static unsigned char pixel_data[128 * 39 * 3 + 1];
};

}

#endif  // GUA_NOISE_TEXTURE_HPP
