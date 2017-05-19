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

#ifndef GUA_TV_3_NODE_HPP
#define GUA_TV_3_NODE_HPP

// guacamole headers
#include <exception>
#include <gua/renderer/TV_3.hpp>
#include <gua/renderer/Material.hpp>

#include <gua/node/GeometryNode.hpp>

#include <unordered_set>

namespace gua {

class TV_3Resource;
class TV_3Loader;

namespace node {

/**
 * This class is used to represent pointcloud in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_TV_3_DLL TV_3Node : public GeometryNode
{
public:
  friend class ::gua::TV_3Loader;

  // c'tor
  TV_3Node(std::string const& node_name,
           std::string const& geometry_description = "gua_default_geometry",
           std::string const& geometry_file_path = "gua_no_path_specified",
           std::shared_ptr<Material> const& material = std::shared_ptr<Material>(),
           math::mat4 const& transform = math::mat4::identity());

public:  // method override

public:  // methods

  std::shared_ptr<TV_3Resource> const& get_geometry() const;

 // /*virtual*/ math::mat4 get_world_transform() const override;

  std::string const& get_geometry_description() const;
  void               set_geometry_description(std::string const& v);

  std::string const& get_geometry_file_path() const;

  std::shared_ptr<Material> const& get_material() const;
  void               set_material(std::shared_ptr<Material> const& material);

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

  std::shared_ptr<TV_3Resource> geometry_;
  std::string                   geometry_description_;
  std::string                   geometry_file_path_;
  bool                          geometry_changed_;

  std::shared_ptr<Material>     material_;
  bool                          material_changed_;
};

}  // namespace node {
}  // namespace gua {

#endif  // GUA_TV_3_NODE_HPP
