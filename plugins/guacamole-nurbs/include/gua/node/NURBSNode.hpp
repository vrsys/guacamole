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

namespace gua {

class NURBSResource;

namespace node {

/**
* This class is used to represent NURBS geometry in the SceneGraph.
*
* \ingroup gua_scenegraph
*/
class GUA_NURBS_DLL NURBSNode : public GeometryNode
{
  friend class NURBSLoader;

private : // c'tor

  NURBSNode(std::string const& node_name,
            std::string const& geometry = "gua_default_geometry",
            Material const& material = Material(),
            math::mat4  const& transform = math::mat4::identity());

public: // methods

  std::shared_ptr<NURBSResource> const& get_geometry() const;

  std::string const&  get_geometry_description() const;
  void                set_geometry_description(std::string const& v);

  Material const&     get_material() const;
  Material&           get_material();
  void                set_material(Material const& material);

public: // render configuration

  float max_pre_tesselation() const;
  void  max_pre_tesselation(float t);

  float max_final_tesselation() const;
  void  max_final_tesselation(float t);

public: // virtual/override methods

  void ray_test_impl(Ray const& ray,
                     int options,
                     Mask const& mask,
                     std::set<PickResult>& hits) override;

  void update_bounding_box() const override;

  void update_cache() override;

  void accept(NodeVisitor& visitor) override;

protected:

  std::shared_ptr<Node> copy() const override;

private : // attributes e.g. special attributes for drawing

  std::shared_ptr<NURBSResource>  geometry_;
  std::string                     geometry_description_;
  bool                            geometry_changed_;

  Material                        material_;
  bool                            material_changed_;

  float                           max_tess_level_pre_pass_; 
  float                           max_tess_level_final_pass_;

};

} // namespace node {
} // namespace gua {

#endif  // GUA_NURBS_NODE_HPP
