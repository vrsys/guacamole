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

#ifndef GUA_DYNAMIC_GEOMETRY_NODE_HPP
#define GUA_DYNAMIC_GEOMETRY_NODE_HPP

// guacamole headers
#include <gua/node/GeometryNode.hpp>

#include <gua/utils/DynamicGeometry.hpp>

namespace gua
{
class DynamicGeometryResource;
class DynamicGeometryLoader;
class GeometryDescription;

namespace node
{
/**
 * This class is used to represent 3D dynamic geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL DynamicGeometryNode : public GeometryNode
{
  public: // typedef/enums/friends
    friend class ::gua::DynamicGeometryLoader;

    DynamicGeometryNode(std::string const& node_name = "",
                        std::string const& geometry_description = "gua_default_geometry",
                        std::shared_ptr<Material> const& material = nullptr,
                        math::mat4 const& transform = math::mat4::identity());

  public: // methods
    /**
     * Get the string referring to an entry in guacamole's GeometryDatabase.
     */
    std::string const& get_geometry_description() const;

    /**
     * Set the string referring to an entry in guacamole's GeometryDatabase.
     */
    void set_geometry_description(std::string const& geometry_description);

    std::shared_ptr<Material> const& get_material() const;
    void set_material(std::shared_ptr<Material> const& material);

    inline bool get_render_to_gbuffer() const { return render_to_gbuffer_; }
    inline void set_render_to_gbuffer(bool enable) { render_to_gbuffer_ = enable; }

    inline bool get_render_to_stencil_buffer() const { return render_to_stencil_buffer_; }
    inline void set_render_to_stencil_buffer(bool enable) { render_to_stencil_buffer_ = enable; }

    inline bool get_render_volumetric() const { return render_volumetric_; }
    inline void set_render_volumetric(bool enable) { render_volumetric_ = enable; }

    inline bool get_render_vertices_as_points() const { return render_vertices_as_points_; }
    inline void set_render_vertices_as_points(bool enable) { render_vertices_as_points_ = enable; }

    inline float get_screen_space_point_size() const { return screen_space_point_size_; }
    inline void set_screen_space_point_size(float point_size) { screen_space_point_size_ = /*std::max(1.0f, std::min(10.0f, */ point_size /*))*/; }

    inline float get_screen_space_line_width() const { return screen_space_line_width_; }
    inline void set_screen_space_line_width(float line_width) { screen_space_line_width_ = std::max(1.0f, std::min(10.0f, line_width)); }

    inline bool get_was_created_empty() const { return was_created_empty_; }
    inline void set_was_created_empty(bool was_created_empty) { was_created_empty_ = was_created_empty; }

    // ephra
    /* inline void set_line_strip_render_mode(scm::gl::primitive_topology const& mode) {
        if

      }
    */

    void set_empty() { was_created_empty_ = true; }

    void compute_consistent_normals();

    void enqueue_vertex(float x,
                        float y,
                        float z,
                        float col_r = 0.0f,
                        float col_g = 0.0f,
                        float col_b = 0.0f,
                        float col_a = 1.0f,
                        float thickness = 1.0f //,
                        // float nor_x = 0.0f, float nor_y = 1.0f, float nor_z = 0.0f
    );

    void push_vertex(DynamicGeometry::Vertex const& dynamic_geometry_vertex);

    void push_vertex(float x,
                     float y,
                     float z,
                     float col_r = 0.0f,
                     float col_g = 0.0f,
                     float col_b = 0.0f,
                     float col_a = 1.0f,
                     float thickness = 1.0f //,
                     // float nor_x = 0.0f, float nor_y = 1.0f, float nor_z = 0.0f
    );

    void pop_back_vertex();
    void pop_front_vertex();

    void clear_vertices();

    virtual void forward_queued_vertices();

    virtual void compile_buffer_string(std::string& buffer_string) =0;
    virtual void uncompile_buffer_string(std::string const& buffer_string) =0;

    /**
     * Implements ray picking for a triangular mesh
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    /**
     * Updates bounding box by accessing the ressource in the databse
     */
    void update_bounding_box() const override;

    void update_cache() override;

    virtual std::shared_ptr<DynamicGeometryResource> const& get_geometry() const;

    bool get_trigger_update() const { return trigger_update_; }
    void set_trigger_update(bool trigger_update) { trigger_update_ = trigger_update; }

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the GeometryNode's data.
     */
    void accept(NodeVisitor& visitor) override;

  protected:
    virtual std::shared_ptr<Node> copy() const = 0;
    // virtual void set_geometry(std::shared_ptr<DynamicGeometryResource> res);

    // std::shared_ptr<DynamicGeometryResource> geometry_;
    std::string geometry_description_;
    bool geometry_changed_;
    bool was_created_empty_;
    bool trigger_update_;

    std::vector<scm::math::vec3f> queued_positions_;
    std::vector<scm::math::vec4f> queued_colors_;
    std::vector<float> queued_thicknesses_;
    std::shared_ptr<DynamicGeometryResource> geometry_;

    std::shared_ptr<Material> material_;
    bool render_to_gbuffer_;
    bool render_to_stencil_buffer_;

    bool render_volumetric_;
    bool render_vertices_as_points_;

    float screen_space_line_width_;
    float screen_space_point_size_;

  private: // methods
    virtual void update_geometry_cache(::gua::GeometryDescription const& desc) = 0;

  private: // attributes e.g. special attributes for drawing
           // std::string                        geometry_description_;
           // bool                               geometry_changed_;

    // bool                              was_created_empty_;

    // bool                              trigger_update_;

    // std::vector<scm::math::vec3f> queued_positions_;
    // std::vector<scm::math::vec4f> queued_colors_;
    // std::vector<float> queued_thicknesses_;
    // std::vector<scm::math::vec3f> queued_normals_;
};

} // namespace node
} // namespace gua

#endif // GUA_LINESTRIP_NODE_HPP
