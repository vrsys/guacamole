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

#ifndef GUA_PLOD_NODE_HPP
#define GUA_PLOD_NODE_HPP

// guacamole headers
#include <gua/renderer/PLOD.hpp>
#include <gua/renderer/Material.hpp>

#include <gua/node/GeometryNode.hpp>

#include <unordered_set>

namespace gua {

class PLODResource;
class PLODLoader;

namespace node {

/**
 * This class is used to represent pointcloud in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_PLOD_DLL PLODNode : public GeometryNode
{
public:
  friend class ::gua::PLODLoader;

  // c'tor
  PLODNode(std::string const& node_name,
           std::string const& geometry_description = "gua_default_geometry",
           std::string const& geometry_file_path = "gua_no_path_specified",
           std::shared_ptr<Material> const& material = std::shared_ptr<Material>(),
           math::mat4 const& transform = math::mat4::identity(),
           float const importance = 1.0f,
           bool const enable_backface_culling_by_normal = false);

public:  // methods

  std::shared_ptr<PLODResource> const& get_geometry() const;

  std::string const& get_geometry_description() const;
  void               set_geometry_description(std::string const& v);

  std::string const& get_geometry_file_path() const;

  std::shared_ptr<Material> const& get_material() const;
  void               set_material(std::shared_ptr<Material> const& material);

  float              get_importance();
  void               set_importance(float const importance);

  bool               get_enable_backface_culling_by_normal();
  void               set_enable_backface_culling_by_normal(bool const enable_backface_culling);

public:
  /**
  * Implements ray picking for a point cloud
  */
  void ray_test_impl(Ray const& ray,
                     int options,
                     Mask const& mask,
                     std::set<PickResult>& hits) override;

  void update_bounding_box() const override;

  void update_cache() override;

  void accept(NodeVisitor& visitor) override;

protected:

  std::shared_ptr<Node> copy() const override;

private:  // attributes e.g. special attributes for drawing

  std::shared_ptr<PLODResource> geometry_;
  std::string                   geometry_description_;
  std::string                   geometry_file_path_;
  bool                          geometry_changed_;

  std::shared_ptr<Material>     material_;
  bool                          material_changed_;

  float                         importance_;
  bool                          enable_backface_culling_by_normal_;
};

}  // namespace node {
}  // namespace gua {

#endif  // GUA_PLOD_NODE_HPP
