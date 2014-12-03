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

#ifndef GUA_SKELETAL_ANIMATION_NODE_HPP
#define GUA_SKELETAL_ANIMATION_NODE_HPP

// guacamole headers
#include <gua/node/GeometryNode.hpp>
#include <gua/renderer/SkeletalAnimationDirector.hpp>

namespace gua {

class SkeletalAnimationRessource;
class SkeletalAnimationLoader;

namespace node {

/**
 * This class is used to represent polygonal geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL SkeletalAnimationNode : public GeometryNode {

public : // typedef/enums/friends

  friend class ::gua::SkeletalAnimationLoader;

private : // c'tor / d'tor

  SkeletalAnimationNode(std::string const& node_name,
              std::vector<std::string> const& geometry_description,
              std::vector<Material> const& materials,
              std::shared_ptr<SkeletalAnimationDirector> animation_director = nullptr,
              math::mat4 const& transform = math::mat4::identity());

public : // methods

  /**
  * Get the string referring to an entry in guacamole's GeometryDatabase.
  */
  //TODO
  //std::string const& get_geometry_description() const;

  /**
  * Set the string referring to an entry in guacamole's GeometryDatabase.
  */
  //TODO
  //void set_geometry_description(std::string const& geometry_description);

  /**
  * A string referring to an entry in guacamole's MaterialShaderDatabase.
  */
  //TODO
  //Material const& get_material() const;
  //TODO
  std::vector<Material>&       get_materials();

  //TODO
  //void            set_material(Material const& material);

  /**
  * Implements ray picking for a triangular mesh
  */
  void ray_test_impl(Ray const& ray,
                     int options,
                     Mask const& mask,
                     std::set<PickResult>& hits) override;

  /**
  * Updates bounding box by accessing the ressource in the databse
  */
  void update_bounding_box() const override;

  void update_cache() override;

  std::vector<std::shared_ptr<SkeletalAnimationRessource>> const& get_geometries() const;
  
  std::shared_ptr<SkeletalAnimationDirector> const& get_director() const;



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

 private:  // attributes e.g. special attributes for drawing


  //std::shared_ptr<SkeletalAnimationRessource> geometry_;
  //std::string                       geometry_description_;
  std::vector<std::shared_ptr<SkeletalAnimationRessource>> geometries_;
  std::vector<std::string>          geometry_descriptions_;
  bool                              geometry_changed_;

  //Material                          material_;
  std::vector<Material>             materials_;
  bool                              material_changed_;

  std::shared_ptr<SkeletalAnimationDirector> animation_director_;


};

} // namespace node {
} // namespace gua {

#endif  // GUA_SKELETAL_ANIMATION_NODE_HPP
