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

#ifndef GUA_TRIMESH_NODE_HPP
#define GUA_TRIMESH_NODE_HPP

// guacamole headers
#include <gua/node/GeometryNode.hpp>

namespace gua
{
class TriMeshRessource;
class TriMeshLoader;

namespace node
{
/**
 * This class is used to represent polygonal geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL TriMeshNode : public GeometryNode
{
  public: // typedef/enums/friends
    friend class ::gua::TriMeshLoader;

    TriMeshNode(std::string const& node_name = "",
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

    virtual std::size_t num_grouped_faces() const override; 

    /**
     * Implements ray picking for a triangular mesh
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    /**
     * Updates bounding box by accessing the ressource in the databse
     */
    void update_bounding_box() const override;

    void update_cache() override;

    std::shared_ptr<TriMeshRessource> const& get_geometry() const;

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * \param visitor  A visitor to process the GeometryNode's data.
     */
    void accept(NodeVisitor& visitor) override;

    inline virtual std::string get_type_string() const {return "<TriMeshNode>";}

  protected:
    std::shared_ptr<Node> copy() const override;

  private: // attributes e.g. special attributes for drawing
    std::shared_ptr<TriMeshRessource> geometry_;
    std::string geometry_description_;
    bool geometry_changed_;

    std::shared_ptr<Material> material_;
    bool render_to_gbuffer_;
    bool render_to_stencil_buffer_;
};

} // namespace node
} // namespace gua

#endif // GUA_TRIMESH_NODE_HPP
