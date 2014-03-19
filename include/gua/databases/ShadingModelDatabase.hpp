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

#ifndef GUA_SHADING_MODEL_DATA_BASE_HPP
#define GUA_SHADING_MODEL_DATA_BASE_HPP

// guacamole headers
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>
#include <gua/renderer/ShadingModel.hpp>

namespace gua {

/**
 * A data base for shading models.
 *
 * This Database stores shading model data. It can be accessed via string
 * identifiers.
 *
 * \ingroup gua_databases
 */
class GUA_DLL ShadingModelDatabase : public Database<ShadingModel>,
                                     public Singleton<ShadingModelDatabase> {
 public:

  /**
   * Pre-loads some shading models.
   *
   * This method loads gsd shading models to the data base.
   *
   * \param directory    An absolute or relative path to the
   *                                  directory containing gsd files.
   */
  static void load_shading_models_from(std::string const& directory);

  static void load_shading_model(std::string const& filename);

  void reload_all();

  friend class Singleton<ShadingModelDatabase>;

 private:
  // this class is a Singleton --- private c'tor and d'tor
  ShadingModelDatabase() {}
  ~ShadingModelDatabase() {}

};

}

#endif  // GUA_SHADING_MODEL_DATA_BASE_HPP
