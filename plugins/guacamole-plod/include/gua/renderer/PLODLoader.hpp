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

#ifndef GUA_PLOD_LOADER_HPP
#define GUA_PLOD_LOADER_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/databases/Database.hpp>

// external headers
#include <string>
#include <list>
#include <memory>

namespace gua {

class Node;
class InnerNode;
class GeometryNode;
class PLODRessource;

class PLODLoader : public GeometryLoader {
 public:

  enum Flags {
    DEFAULTS = 0,
    MAKE_PICKABLE = 1 << 0,
    NORMALIZE_POSITION = 1 << 1,
    NORMALIZE_SCALE = 1 << 2
  };

  /**
   * Default constructor.
   *
   * Constructs a new and empty PLODLoader.
   */
  PLODLoader();

  /**
   * Constructor from a file.
   *
   * Creates a new PLODLoader from a given file.
   *
   * \param node_name  Name of the scenegraph node.
   * \param file_name  The file to load the pointclouds data from.
   * \param flags      Loading flags
   */
  std::shared_ptr<gua::node::Node> create_geometry_from_file(
      std::string const& node_name,
      std::string const& file_name,
      unsigned flags = DEFAULTS);

  bool is_supported(std::string const& file_name) const override;

  size_t get_upload_budget_in_mb() const;
  size_t get_render_budget_in_mb() const;
  size_t get_out_of_core_budget_in_mb() const;

  void   set_upload_budget_in_mb(const size_t upload_budget);
  void   set_render_budget_in_mb(const size_t render_budget);
  void   set_out_of_core_budget_in_mb(const size_t out_of_core_budget);

 private:

  boost::unordered_set<std::string> _supported_file_extensions;

};

}

#endif  // GUA_PLOD_LOADER_HPP
