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

#ifndef GUA_TEMPORAL_SVO_NODE_HPP
#define GUA_TEMPORAL_SVO_NODE_HPP

// guacamole headers
#include <exception>
#include <gua/renderer/TemporalSVO.hpp>
#include <gua/renderer/TemporalSVOResource.hpp>
#include <gua/renderer/Material.hpp>

#include <gua/node/GeometryNode.hpp>

#include <unordered_set>

namespace gua
{
class TemporalSVOResource;
class TemporalSVOLoader;

namespace node
{
/**
 * This class is used to represent pointcloud in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_TEMPORAL_SVO_DLL TemporalSVONode : public GeometryNode
{
  public:

    enum PlaybackMode
    {
        NONE = TemporalSVOResource::NONE,
        FORWARD = TemporalSVOResource::FORWARD,
        BACKWARD = TemporalSVOResource::BACKWARD,

        PLAYBACK_MODE_COUNT
    };

    friend class ::gua::TemporalSVOLoader;

    // c'tor
    TemporalSVONode(std::string const& node_name,
             std::string const& geometry_description = "gua_default_geometry",
             std::string const& geometry_file_path = "gua_no_path_specified",
             std::shared_ptr<Material> const& material = std::shared_ptr<Material>(),
             math::mat4 const& transform = math::mat4::identity());

  public: // method override
  public: // methods
    std::shared_ptr<TemporalSVOResource> const& get_geometry() const;

    // /*virtual*/ math::mat4 get_world_transform() const override;

    std::string const& get_geometry_description() const;
    void set_geometry_description(std::string const& v);

    std::string const& get_geometry_file_path() const;

    std::shared_ptr<Material> const& get_material() const;
    void set_material(std::shared_ptr<Material> const& material);

  public:
    /**
     * Implements ray picking for a point cloud
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    void update_bounding_box() const override;

    void update_cache() override;

    void accept(NodeVisitor& visitor) override;

    int get_num_time_steps() const { return geometry_->get_num_volume_time_steps(); }

    void set_playback_mode(PlaybackMode playback_mode) { geometry_->set_playback_mode(static_cast<TemporalSVOResource::PlaybackMode>(playback_mode)); }
    PlaybackMode get_playback_mode() const { return static_cast<TemporalSVONode::PlaybackMode>(geometry_->get_playback_mode()); }

    void set_playback_fps(float playback_fps) { geometry_->set_playback_fps(playback_fps); }
    bool get_playback_fps() const { return geometry_->get_playback_fps(); }

    void set_time_cursor_pos(float const time_cursor_pos) const { geometry_->set_time_cursor_pos(time_cursor_pos); }
    float get_time_cursor_pos() const { return geometry_->get_time_cursor_pos(); }

    //RenderMode get_render_mode() const { return render_mode_; }
    //void set_render_mode(RenderMode const render_mode) { render_mode_ = render_mode; }

    float get_iso_value() const { return iso_value_; }
    void set_iso_value(float iso_value) { iso_value_ = iso_value; }

    std::shared_ptr<Node> copy() const override;

  private: // attributes e.g. special attributes for drawing
    std::shared_ptr<TemporalSVOResource> geometry_;
    std::string geometry_description_;
    std::string geometry_file_path_;
    bool geometry_changed_;

    std::shared_ptr<Material> material_;
    bool material_changed_;

    float iso_value_ = 0.0;

    //SpatialFilterMode spatial_filter_mode_{SpatialFilterMode::S_NEAREST};
    //TemporalFilterMode temporal_filter_mode_{TemporalFilterMode::T_NEAREST};
};

} // namespace node
} // namespace gua

#endif // GUA_TEMPORAL_SVO_NODE_HPP
