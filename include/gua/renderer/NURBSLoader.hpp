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

#ifndef GUA_NURBS_LOADER_HPP
#define GUA_NURBS_LOADER_HPP

// guacamole headers
#include <gua/renderer/LoaderBase.hpp>

// external headers
#include <boost/unordered_set.hpp>

namespace gua {

class Node;

/**
 * Loads NURBS files and creates NURBS nodes.
 *
 * This class can load NURBS data from files and display them in multiple
 * contexts.
 */
class NURBSLoader : public LoaderBase {
 public:

  /**
   * Default constructor.
   *
   * Constructs a new and empty NURBSLoader.
   */
  NURBSLoader();

  /**
   * Constructor from a file.
   *
   * Creates a new NURBS node from a given file.
   *
   * \param file_name        The file to load the NURBS data from.
   * \param unsigned         Special flag
   */
  /* virtual */ std::shared_ptr<Node> load(std::string const& file_name,
                                           unsigned flags);

  /* virtual */ bool is_supported(std::string const& file_name) const;

 private:

  boost::unordered_set<std::string> _supported_file_extensions;

};

}

#endif  // GUA_NURBS_LOADER_HPP
