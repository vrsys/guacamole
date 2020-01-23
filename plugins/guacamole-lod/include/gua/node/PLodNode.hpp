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

    void set_attribute_to_visualize_index(int attribute_to_visualize_index);
    int get_attribute_to_visualize_index() const;

    //update the time cursor with the elapsed seconds since the last frame
    void update_time_cursor(float elapsed_frame_time_seconds);

    void set_time_cursor_position(float time_cursor);
    float get_time_cursor_position() const;

    float get_current_time_step() const;

    void set_enable_time_series_deformation(bool deformation);
    bool get_enable_time_series_deformation() const;
    void set_enable_time_series_coloring(bool coloring);
    bool get_enable_time_series_coloring() const;

    void set_enable_automatic_playback(bool enable_automatic_playback);
    bool get_enable_automatic_playback() const;

    void set_enable_temporal_interpolation(bool enable_temporal_interpolation);
    bool get_enable_temporal_interpolation() const;

    void set_time_series_playback_speed(float time_series_playback_speed);
    float get_time_series_playback_speed() const;

    void set_time_series_deform_factor(float time_series_deform_factor);
    float get_time_series_deform_factor() const;

    void set_attribute_color_mix_in_factor(float attribute_color_mix_in_factor);
    float get_attribute_color_mix_in_factor() const;

    void set_has_provenance_attributes(bool has_provenance_attributes);
    bool get_has_provenance_attributes() const;

    void set_active_time_series_index(unsigned int time_series_index);
    int get_active_time_series_index() const;

    scm::math::mat4f get_active_time_series_transform() const;

    void set_time_series_data_descriptions(std::vector<std::string> const& time_series_data_descriptions);
    std::vector<std::string> get_time_series_data_descriptions() const;

    int get_number_of_simulation_positions() const;
    std::vector<scm::math::vec3f> get_current_simulation_positions() const;
  public:
    /**
     * Implements ray picking for a point cloud
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    void update_bounding_box() const override;

    void update_cache() override;

    void accept(NodeVisitor& visitor) override;

    void bind_time_series_data_to(RenderContext& ctx, std::shared_ptr<ShaderProgram>& current_program);

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


    bool enable_time_series_coloring_ = true;
    bool enable_time_series_deformation_ = true;
    bool enable_automatic_playback_ = true;
    float time_series_playback_speed_ = 1.0f;
    float time_series_deform_factor_  = 1.0f;

    bool enable_temporal_interpolation_ = true;

    float attribute_color_mix_in_factor_ = 0.7f;

    int attribute_to_visualize_index_ = 0;

    int active_time_series_data_description_index_ = 0;
    std::vector<std::string> associated_time_series_data_descriptions_;

    bool has_provenance_attributes_ = false;
};

} // namespace node
} // namespace gua

#endif // GUA_P_LOD_NODE_HPP
