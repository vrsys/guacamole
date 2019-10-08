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

#ifndef GUA_DYNAMIC_TRIANGLE_NODE_HPP
#define GUA_DYNAMIC_TRIANGLE_NODE_HPP

// guacamole headers
#include <gua/node/DynamicGeometryNode.hpp>

#include <gua/utils/DynamicTriangle.hpp>

namespace gua
{
class DynamicGeometryResource;
class DynamicTriangleResource;
class DynamicGeometryLoader;
class DynamicTriangleLoader;

namespace node
{
/**
 * This class is used to represent 3D dynamic geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL DynamicTriangleNode : public DynamicGeometryNode
{
  public: // typedef/enums/friends
    friend class ::gua::DynamicGeometryLoader;

    DynamicTriangleNode(std::string const& node_name = "",
                        std::string const& geometry_description = "gua_default_geometry",
                        std::shared_ptr<Material> const& material = nullptr,
                        math::mat4 const& transform = math::mat4::identity());

  public: // methods
    // ephra
    /* inline void set_line_strip_render_mode(scm::gl::primitive_topology const& mode) {
        if

      }
    */

    void enqueue_vertex(float x, float y, float z, float col_r = 0.0f, float col_g = 0.0f, float col_b = 0.0f, float col_a = 1.0f, float thickness = 1.0f, float u = 0.0f, float v = 1.0f);

    void push_vertex(DynamicTriangle::TriVertex const& dynamic_tri_vertex);

    void push_vertex(float x, float y, float z, float col_r = 0.0f, float col_g = 0.0f, float col_b = 0.0f, float col_a = 1.0f, float thickness = 1.0f, float u = 0.0f, float v = 1.0f);

    void
    update_vertex(int vertex_idx, float x, float y, float z, float col_r = 0.0f, float col_g = 0.0f, float col_b = 0.0f, float col_a = 1.0f, float thickness = 1.0f, float u = 0.0f, float v = 0.0f);

    void clear_vertices();

    void forward_queued_vertices() override;

    void compile_buffer_string(std::string& buffer_string) override;
    void uncompile_buffer_string(std::string const& buffer_string) override;

    /**
     * Implements ray picking for a triangular mesh
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    /**
     * Updates bounding box by accessing the ressource in the databse
     */
    void update_bounding_box() const override;

    std::shared_ptr<DynamicGeometryResource> const& get_geometry() const override;

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
    std::shared_ptr<Node> copy() const override;

  private: // methods
    void update_geometry_cache(::gua::GeometryDescription const& desc) override;

    // void set_geometry(std::shared_ptr<DynamicGeometryResource> res) override;

  private: // attributes e.g. special attributes for drawing
    // std::shared_ptr<DynamicTriangleResource> geometry_;

    std::vector<scm::math::vec2f> queued_uvs_;
};

} // namespace node
} // namespace gua

#endif // GUA_DYNAMIC_TRIANGLE_NODE_HPP
