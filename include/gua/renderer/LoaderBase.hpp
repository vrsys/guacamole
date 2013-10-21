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

#ifndef GUA_LOADER_BASE_HPP
#define GUA_LOADER_BASE_HPP

// guacamole headers

// external headers
#include <memory>
#include <string>
#include <vector>

namespace gua {

class Node;

/**
 * Virtual base class for any file loader
 *
 * This class provides the interface for any file loader generating nodes
 */
class LoaderBase {
 public:

  /**
   * Default constructor.
   *
   * Constructs a new and empty LoaderBase.
   */
  LoaderBase() {}

  /**
   * Destructor.
   *
   * Deletes the LoaderBase
   */
  virtual ~LoaderBase() {}

  /**
   * Interface for loading from a file.
   *
   * Creates a new Node from a file
   *
   * \param flags        TODO: what does flags?
   */
  virtual std::shared_ptr<Node> load(std::string const& file_name,
                                     std::string const& fallback_material,
                                     unsigned flags) = 0;

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

#endif  // GUA_LOADER_BASE_HPP
