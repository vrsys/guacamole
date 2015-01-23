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
#include <gua/renderer/PLOD.hpp>

// external headers
#include <unordered_set>
#include <memory>

namespace gua {

  class Material;

namespace node {
  class Node;
  class PLODNode;
}

/**
 * Loads *.kdn and *.lod files and creates PLOD nodes.
 *
 * This class can load PLOD data from files and display them in multiple
 * contexts.
 */

class GUA_PLOD_DLL PLODLoader {
 public:

  enum Flags {
    DEFAULTS = 0,
    MAKE_PICKABLE = 1 << 0,
    NORMALIZE_POSITION = 1 << 1,
    NORMALIZE_SCALE = 1 << 2
  };


  PLODLoader();

public:

  std::shared_ptr<node::PLODNode> load_geometry(std::string const& file_name,
                                                unsigned flags = DEFAULTS);

  std::shared_ptr<node::PLODNode> load_geometry(std::string const& node_name,
                                                std::string const& file_name,
                                                std::shared_ptr<Material> const& fallback_material,
                                                unsigned flags =  DEFAULTS);

  void apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material) const;

  /**
   * PLOD-lib specific configuration methods. Might be moved into a separate object later.
   *
   */
  size_t get_upload_budget_in_mb() const;
  size_t get_render_budget_in_mb() const;
  size_t get_out_of_core_budget_in_mb() const;
  float  get_error_treshold() const;
  float  get_importance(std::string const& file_name) const;

  void   set_upload_budget_in_mb(size_t const upload_budget);
  void   set_render_budget_in_mb(size_t const render_budget);
  void   set_out_of_core_budget_in_mb(size_t const out_of_core_budget);
  void   set_error_threshold(float const error_threshold);
  void   set_importance(std::string const& file_name, float const importance);

  bool is_supported(std::string const& file_name) const;

private:

  std::unordered_set<std::string> _supported_file_extensions;

};

}

#endif  // GUA_PLOD_LOADER_HPP
