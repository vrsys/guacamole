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

#ifndef IGS_GEOMETRY_BASE_HPP
#define IGS_GEOMETRY_BASE_HPP

#include <gua/utils/Singleton.hpp>
#include <gua/renderer/DataBase.hpp>
#include <gua/renderer/IGSGeometry.hpp>
#include <gua/renderer/enums.hpp>

/**
 * A data base for meshes.
 *
 * This DataBase stores geometry data. It can be accessed via string
 * identifiers.
 */

namespace gua {

class IGSGeometryBase : public DataBase<IGSGeometry>,
                        public Singleton<IGSGeometryBase> {
 public:
  /**
   * Pre-loads some meshes.
   *
   * This method loads some default meshes to the data base. For example
   * a simple cube ("cube"), the famous Utah Teapot ("teapot") and
   * Suzanne from Blender ("monkey").
   */
  static void load_objects_from(std::string const& path_to_objects);

  friend class Singleton<IGSGeometryBase>;

  static void set_fill_mode(IGSGeometryMode mode);

  //Program Ist Pass
  mutable std::vector<scm::gl::program_ptr> _programI;

  void upload_to(RenderContext context);

  mutable std::mutex upload_mutex_;

 private:
  IGSGeometryBase();
  ~IGSGeometryBase() {}
  ;
};

}

#endif  // GEOMETRY_BASE_HPP
