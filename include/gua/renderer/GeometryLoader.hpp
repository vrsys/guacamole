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

#ifndef GUA_LOADER_FACTORY_HPP
#define GUA_LOADER_FACTORY_HPP

// guacamole headers
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>
#include <gua/renderer/Geometry.hpp>
#include <gua/renderer/LoaderBase.hpp>

namespace gua {

class Node;

/**
 * A data base for meshes.
 *
 * This DataBase stores geometry data. It can be accessed via string
 * identifiers.
 */
class GUA_DLL GeometryLoader {
 public:

  enum Flags {
    DEFAULTS            = 0,
    LOAD_MATERIALS      = 1<<0,
    OPTIMIZE_GEOMETRY   = 1<<1,
    MAKE_PICKABLE       = 1<<2,
    NORMALIZE_POSITION  = 1<<3,
    NORMALIZE_SCALE     = 1<<4
  };

  GeometryLoader();

  virtual ~GeometryLoader();

  std::shared_ptr<Node> load_geometry(std::string const& file_name, unsigned flags = DEFAULTS);

  std::shared_ptr<Node> create_geometry_from_file(std::string const& node_name,
                                        std::string const& file_name,
                                        std::string const& fallback_material,
                                        unsigned flags = DEFAULTS);

  std::shared_ptr<Node> create_volume_from_file(std::string const& node_name,
																std::string const& file_name,
																unsigned flags = DEFAULTS);
	  

  std::shared_ptr<Node> create_vvolume_from_file(std::string const&	node_name,
															  std::string const&	vfile_name,
															  unsigned				flags = DEFAULTS,
															  scm::size_t			vol_hdd_cache_size = 2147483648,
															  scm::size_t			vol_gpu_cache_size = 536870912);


 private:

  void apply_fallback_material(std::shared_ptr<Node> const& root, std::string const& fallback_material) const;

  static std::unordered_map<std::string, std::shared_ptr<Node>> loaded_files_;

  std::string parent_material_name_;
  std::list<LoaderBase*> fileloaders_;
};

}

#endif  // GUA_LOADER_FACTORY_HPP
