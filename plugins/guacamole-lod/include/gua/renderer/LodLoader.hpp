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

#ifndef GUA_LOD_LOADER_HPP
#define GUA_LOD_LOADER_HPP

// guacamole headers
#include <gua/renderer/Lod.hpp>
#include <gua/scenegraph/PickResult.hpp>
// external headers
#include <set>
#include <unordered_set>
#include <memory>

namespace gua
{
class Material;

namespace node
{
class Node;
class PLodNode;
class MLodNode;
} // namespace node

/**
 * Loads *.kdn and *.lod files and creates Lod nodes.
 *
 * This class can load Lod data from files and display them in multiple
 * contexts.
 */

class GUA_LOD_DLL LodLoader
{
  public:
    enum Flags
    {
        DEFAULTS = 0,
        MAKE_PICKABLE = 1 << 0,
        NORMALIZE_POSITION = 1 << 1,
        NORMALIZE_SCALE = 1 << 2
    };

    LodLoader();

  public:

    std::vector<std::shared_ptr<node::PLodNode>> load_lod_pointclouds_from_vis_file(std::string const& vis_file_name, unsigned flags);

    std::vector<std::shared_ptr<node::PLodNode>> load_lod_pointclouds_from_vis_file(std::string const& vis_file_name, std::shared_ptr<Material> const& fallback_material, unsigned flags);
    
    std::shared_ptr<node::PLodNode> load_lod_pointcloud(std::string const& file_name, unsigned flags = DEFAULTS);

    std::shared_ptr<node::PLodNode> load_lod_pointcloud(std::string const& node_name, std::string const& file_name, std::shared_ptr<Material> const& fallback_material, unsigned flags = DEFAULTS);

    std::shared_ptr<node::MLodNode> load_lod_trimesh(std::string const& file_name, unsigned flags = DEFAULTS);

    std::shared_ptr<node::MLodNode> load_lod_trimesh(std::string const& node_name, std::string const& file_name, std::shared_ptr<Material> const& fallback_material, unsigned flags = DEFAULTS);

    void apply_fallback_material(std::shared_ptr<node::Node> const& root, std::shared_ptr<Material> const& fallback_material) const;

    /**
     * Pointcloud-specific picking methods. Might be moved into a separate object later.
     *
     */
    std::pair<std::string, math::vec3>
    pick_lod_bvh(math::vec3 const& ray_origin, math::vec3 const& ray_forward, float max_distance, std::set<std::string> const& model_filenames, float aabb_scale) const;

    std::set<PickResult> pick_lod_interpolate(math::vec3 const& bundle_origin,
                                              math::vec3 const& bundle_forward,
                                              math::vec3 const& bundle_up,
                                              float bundle_radius,
                                              float max_distance,
                                              unsigned int max_depth,
                                              unsigned int surfel_skip,
                                              float aabb_scale) const;

    /**
     * Lod-lib specific configuration methods. Might be moved into a separate object later.
     *
     */
    size_t get_upload_budget_in_mb() const;
    size_t get_render_budget_in_mb() const;
    size_t get_out_of_core_budget_in_mb() const;

    void set_upload_budget_in_mb(size_t const upload_budget);
    void set_render_budget_in_mb(size_t const render_budget);
    void set_out_of_core_budget_in_mb(size_t const out_of_core_budget);

    bool is_supported_model_file(std::string const& file_name) const;
    bool is_supported_vis_file(std::string const& file_name) const;
  private: // methods
    math::mat4 _load_local_transform(std::string const& filename) const;

  private: // member
    std::unordered_set<std::string> _supported_file_extensions_model_file;
    std::unordered_set<std::string> _supported_file_extensions_vis_file;
};

} // namespace gua

#endif // GUA_LOD_LOADER_HPP
