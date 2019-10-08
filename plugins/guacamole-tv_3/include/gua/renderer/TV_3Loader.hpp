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

#ifndef GUA_TV_3_LOADER_HPP
#define GUA_TV_3_LOADER_HPP

// guacamole headers
#include <gua/renderer/TV_3.hpp>
#include <gua/scenegraph/PickResult.hpp>
// external headers
#include <set>
#include <unordered_set>
#include <unordered_map>
#include <memory>
#include <atomic>

namespace gua
{
class Material;

namespace node
{
class Node;
class TV_3Node;
} // namespace node

/**
 * Loads *.kdn and *.lod files and creates PLOD nodes.
 *
 * This class can load PLOD data from files and display them in multiple
 * contexts.
 */

class GUA_TV_3_DLL TV_3Loader
{
  public:
    enum Flags
    {
        DEFAULTS = 0,
        MAKE_PICKABLE = 1 << 0,
        NORMALIZE_POSITION = 1 << 1,
        NORMALIZE_SCALE = 1 << 2,
        USE_SURFACE_MODE = 1 << 3
    };

    TV_3Loader();

  public:
    std::shared_ptr<node::Node> load_geometry(std::string const& file_name, unsigned flags = DEFAULTS, int const cpu_budget_in_mb = 1024, int const gpu_budget_in_mb = 1024);

    std::shared_ptr<node::Node> create_geometry_from_file(std::string const& node_name,
                                                          std::string const& file_name,
                                                          std::shared_ptr<Material> const& fallback_material,
                                                          unsigned flags = DEFAULTS,
                                                          int const cpu_budget = 1024,
                                                          int const gpu_budget = 1024);

    std::shared_ptr<node::Node>
    create_geometry_from_file(std::string const& node_name, std::string const& file_name, unsigned flags = DEFAULTS, int const cpu_budget = 1024, int const gpu_budget = 1024);

    void apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material, bool no_shared_materials);

    /*
      std::pair<std::string, math::vec3> pick_plod_bvh(math::vec3 const& ray_origin,
                                                       math::vec3 const& ray_forward,
                                                       float max_distance,
                                                       std::set<std::string> const& model_filenames,
                                                        float aabb_scale) const;

      std::set<PickResult> pick_plod_interpolate(math::vec3 const& bundle_origin,
                                                 math::vec3 const& bundle_forward,
                                                 math::vec3 const& bundle_up,
                                                 float bundle_radius,
                                                 float max_distance,
                                                 unsigned int max_depth,
                                                 unsigned int surfel_skip,
                                                 float aabb_scale) const;
    */

    /**
     * TV_3-lib specific configuration methods. Might be moved into a separate object later.
     *
     */
    /*
     size_t get_cpu_budget_in_mb() const;
     size_t get_gpu_budget_in_mb() const;

     void   set_cpu_budget_in_mb(size_t const cpu_budget);
     void   set_gpu_budget_in_mb(size_t const gpu_budget);
   */
    bool is_supported(std::string const& file_name) const;

  private: // methods
    math::mat4 _load_local_transform(std::string const& filename) const;

  private: // member
    std::unordered_set<std::string> _supported_file_extensions;

    static std::unordered_map<std::string, std::shared_ptr<::gua::node::Node>> loaded_files_;
};

} // namespace gua

#endif // GUA_TV_3_LOADER_HPP
