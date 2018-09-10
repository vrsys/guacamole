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

#ifndef GUA_AUX_HPP
#define GUA_AUX_HPP

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/scenegraph/PickResult.hpp>
// external headers
#include <set>
#include <unordered_set>
#include <memory>

namespace lamure {
namespace prov {
  class aux;
}
}

namespace gua {


class GUA_LOD_DLL Aux {
 public:


  Aux();

public:

  void load_aux_file(std::string const& filename);

  void test_wrapping() const;

  const std::string   get_filename() const;

  const uint32_t      get_num_views() const;
  const uint64_t      get_num_sparse_points() const;
  const uint32_t      get_num_atlas_tiles() const;

private: // methods

private: // member
  std::shared_ptr<lamure::prov::aux> _aux;

};

}

#endif  // GUA_AUX_HPP
