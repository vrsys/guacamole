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

#ifndef GUA_P_LOD_NODE_HPP
#define GUA_P_LOD_NODE_HPP

// guacamole headers
#include <exception>
#include <gua/renderer/Lod.hpp>
#include <gua/renderer/Material.hpp>

#include <gua/node/GeometryNode.hpp>

#include <unordered_set>

namespace gua
{
class LodResource;
class LodLoader;

namespace node
{
/**
 * This class is used to represent pointcloud in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_LOD_DLL PLodNode : public GeometryNode
{
  public:
    friend class ::gua::LodLoader;

    // c'tor
    PLodNode(std::string const& node_name,
             std::string const& geometry_description = "gua_default_geometry",
             std::string const& geometry_file_path = "gua_no_path_specified",
             std::shared_ptr<Material> const& material = std::shared_ptr<Material>(),
             math::mat4 const& transform = math::mat4::identity(),
             float const max_surfel_size = 0.2f,
             float const radius_scale = 1.0f,
             float const error_threshold = 2.5f,
             bool const enable_backface_culling_by_normal = false);

  public: // method override
  public: // methods
    std::shared_ptr<LodResource> const& get_geometry() const;

    /*virtual*/ math::mat4 get_world_transform() const override;

    std::string const& get_geometry_description() const;
    void set_geometry_description(std::string const& v);

    std::string const& get_geometry_file_path() const;

    std::shared_ptr<Material> const& get_material() const;
    void set_material(std::shared_ptr<Material> const& material);

    float get_radius_scale() const;
    void set_radius_scale(float scale);

    float get_max_surfel_radius() const;
    void set_max_surfel_radius(float threshold);

    float get_error_threshold() const;
    void set_error_threshold(float threshold);

    bool get_enable_backface_culling_by_normal() const;
    void set_enable_backface_culling_by_normal(bool const enable_backface_culling);

    void set_time_series_data_descriptions(std::vector<std::string> const& time_series_data_descriptions);
    std::vector<std::string> get_time_series_data_descriptions();
    
  public:
    /**
     * Implements ray picking for a point cloud
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    void update_bounding_box() const override;

    void update_cache() override;

    void accept(NodeVisitor& visitor) override;

  protected:
    std::shared_ptr<Node> copy() const override;

  private: // attributes e.g. special attributes for drawing
    std::shared_ptr<LodResource> geometry_;
    std::string geometry_description_;
    std::string geometry_file_path_;
    bool geometry_changed_;

    std::shared_ptr<Material> material_;
    bool material_changed_;

    float radius_scale_;
    float max_surfel_size_;
    float error_threshold_;
    bool enable_backface_culling_by_normal_;


    std::vector<std::string> associated_time_series_data_descriptions_;

};

} // namespace node
} // namespace gua

#endif // GUA_P_LOD_NODE_HPP
