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

#ifndef GUA_RESOURCES_HPP
#define GUA_RESOURCES_HPP

// external headers
#include <vector>
#include <string>

namespace gua {

namespace Resources {

  std::string                       lookup_string(std::string const& file);
  std::string                       lookup_string(std::vector<unsigned char> const& resource);

  std::string                       lookup_shader(std::string const& file);
  std::string                       lookup_shader(std::vector<unsigned char> const& resource);

  std::vector<unsigned char> const& lookup(std::string const& file);

  // generated header
  #include <gua/generated/R.inl>

}
}

#endif  // GUA_RESOURCES_HPP
