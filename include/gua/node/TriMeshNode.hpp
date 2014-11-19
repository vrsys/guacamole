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

namespace gua {

class TriMeshRessource;

namespace node {

/**
 * This class is used to represent polygonal geometry in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL TriMeshNode : public GeometryNode {
 public:  // member

  TriMeshNode(std::string const& name = "",
              std::string const& filename = "gua_default_geometry",
              Material const& material = Material(),
              math::mat4 const& transform = math::mat4::identity());


  /**
  * Get the string referring to an entry in guacamole's GeometryDatabase.
  */
  std::string const& get_filename() const;

  /**
  * Set the string referring to an entry in guacamole's GeometryDatabase.
  */
  void set_filename(std::string const& filename);

  /**
  * A string referring to an entry in guacamole's MaterialShaderDatabase.
  */
  Material const& get_material() const;
  Material&       get_material();
  void            set_material(Material const& material);

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

  std::shared_ptr<TriMeshRessource> const& get_geometry() const;

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

  std::shared_ptr<TriMeshRessource> geometry_;

  std::string filename_;

  Material material_;

  bool filename_changed_;
  bool material_changed_;

};

} // namespace node {
} // namespace gua {

#endif  // GUA_TRIMESH_NODE_HPP
