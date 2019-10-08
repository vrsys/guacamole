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
#ifndef GUA_NURBS_NODE_HPP
#define GUA_NURBS_NODE_HPP

// guacamole headers
#include <gua/renderer/NURBS.hpp>
#include <gua/renderer/Material.hpp>

#include <gua/node/GeometryNode.hpp>

namespace gua
{
class NURBSResource;
class NURBSLoader;

namespace node
{
/**
 * This class is used to represent NURBS geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_NURBS_DLL NURBSNode : public GeometryNode
{
  public:
    friend class NURBSLoader;

  public: // c'tor
    NURBSNode(std::string const& node_name,
              std::string const& geometry_description = "gua_default_geometry",
              std::shared_ptr<Material> const& material = nullptr,
              math::mat4 const& transform = math::mat4::identity());

  public: // methods
    std::shared_ptr<NURBSResource> const& get_geometry() const;

    std::string const& get_geometry_description() const;
    void set_geometry_description(std::string const& v);

    std::shared_ptr<Material> const& get_material() const;
    void set_material(std::shared_ptr<Material> const& material);

  public: // render configuration
    float max_pre_tesselation() const;
    void max_pre_tesselation(float t);

    float max_tesselation_error() const;
    void max_tesselation_error(float t);

    void wireframe(bool enable);
    bool wireframe() const;

    void trimming(bool enable);
    bool trimming() const;

  public: // virtual/override methods
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    void update_bounding_box() const override;

    void update_cache() override;

    void accept(NodeVisitor& visitor) override;

  protected:
    std::shared_ptr<Node> copy() const override;

  private: // attributes e.g. special attributes for drawing
    std::shared_ptr<NURBSResource> geometry_;
    std::string geometry_description_;
    bool geometry_changed_;

    std::shared_ptr<Material> material_;
    bool material_changed_;

    float max_tesselation_error_ = 8.0f;
    float max_pre_tesselation_ = 64.0f;
    bool wireframe_ = false;
    bool trimming_ = true;
};

} // namespace node
} // namespace gua

#endif // GUA_NURBS_NODE_HPP
