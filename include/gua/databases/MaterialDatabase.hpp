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

#ifndef GUA_MATERIAL_DATABASE_HPP
#define GUA_MATERIAL_DATABASE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>
#include <gua/renderer/Material.hpp>

namespace gua {

/**
 * A data base for materials.
 *
 * This Database stores material data. It can be accessed via string
 * identifiers.
 *
 * \ingroup gua_databases
 */
class GUA_DLL MaterialDatabase : public Database<Material>,
                                 public Singleton<MaterialDatabase> {
 public:

  std::mutex update_lock;

  /**
   * Pre-loads some Materials.
   *
   * This method loads gmd materials to the data base.
   *
   * \param directory    An absolute or relative path to the
   *                     directory containing gmd files.
   */
  static void load_materials_from(std::string const& directory);

  static void load_material(std::string const& filename);

  void reload_all();

  friend class Singleton<MaterialDatabase>;

 private:
  // this class is a Singleton --- private c'tor and d'tor
  MaterialDatabase() {}
  ~MaterialDatabase() {}

};

}

#endif  // GUA_MATERIAL_DATABASE_HPP
