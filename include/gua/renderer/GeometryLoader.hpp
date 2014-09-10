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
#include <gua/renderer/GeometryRessource.hpp>

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

   /**
   * Default constructor.
   *
   * Constructs a new and empty LoaderBase.
   */
   GeometryLoader() = default;

   /**
   * Destructor.
   *
   * Deletes the LoaderBase
   */
   virtual ~GeometryLoader() {};

   /**
   * Interface for file support
   *
   * returns if the filename is supported by this loader
   *
   * \param file_name        The file to load the data from.
   */
   virtual bool is_supported(std::string const& file_name) const = 0;

private:

};

}

#endif  // GUA_LOADER_FACTORY_HPP
