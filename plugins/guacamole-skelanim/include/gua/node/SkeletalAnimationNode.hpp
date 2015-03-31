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
#include <gua/renderer/SkinnedMeshResource.hpp>
#include <gua/utils/SkeletalAnimation.hpp>
#include <gua/utils/Bone.hpp>

namespace gua {

// class SkinnedMeshResource;
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

  SkeletalAnimationNode(std::string const& node_name = "",
              std::vector<std::string> const& geometry_description = {},
              std::vector<std::shared_ptr<Material>> const& materials = {},
              std::shared_ptr<Bone> const& = nullptr,
              math::mat4 const& transform = math::mat4::identity());

private : // c'tor / d'tor

public : // methods


  Node* get();

  /**
  * Get the strings referring to an entry in guacamole's GeometryDatabase.
  */
  std::vector<std::string> const& get_geometry_descriptions() const;

  /**
  * Set the string referring to an entry in guacamole's GeometryDatabase.
  */
  void set_geometry_description(std::string const& geometry_description, uint index);

  /**
  * This is only for the multifield handling in avango
  */
  void add_material(std::shared_ptr<Material> const& material);
  /**
  * A string referring to an entry in guacamole's MaterialShaderDatabase.
  */

  std::vector<std::shared_ptr<Material>> const& get_materials() const;

  std::shared_ptr<Material> const&              get_material(uint index) const;

  void            set_material(std::shared_ptr<Material> material, uint index);

  inline void clear_materials() { materials_.clear(); }

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

  std::vector<math::BoundingBox<math::vec3>> get_bone_boxes();

  void update_cache() override;

  std::vector<std::shared_ptr<SkinnedMeshResource>> const& get_geometries() const;

//animation related methods
  void add_animations(aiScene const& scene, std::string const& name);
  void add_animations(fbxsdk_2015_1::FbxScene& scene, std::string const& name);

  std::string const& get_animation_1() const;
  void set_animation_1(std::string const&);

  std::string const& get_animation_2() const;
  void set_animation_2(std::string const&);

  float get_blend_factor() const;
  void set_blend_factor(float f);

  float get_duration(std::string const&) const;

  float get_time_1() const;
  void set_time_1(float);
  float get_time_2() const;
  void set_time_2(float);

  bool has_anims() const;

  std::vector<scm::math::mat4f> const& get_bone_transforms() const;
  void update_bone_transforms();


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

  std::vector<std::shared_ptr<SkinnedMeshResource>> geometries_;
  std::vector<std::string>          geometry_descriptions_;
  bool                              geometry_changed_;

  std::vector<std::shared_ptr<Material>>             materials_;

  bool                              material_changed_;

  // attributes related to animation

  std::map<std::string, int> bone_mapping_; // maps a bone name to its index

  std::shared_ptr<Bone> root_;
  std::shared_ptr<Bone> anim_start_node_;

  std::map<std::string, SkeletalAnimation> animations_;


  bool first_run_;
  bool has_anims_;
  uint num_bones_;

  const static std::string none_loaded;

  float blend_factor_;
  std::string anim_1_;
  std::string anim_2_;
  float anim_time_1_;
  float anim_time_2_;

  std::vector<scm::math::mat4f> bone_transforms_;
};

} // namespace node {
} // namespace gua {

#endif  // GUA_SKELETAL_ANIMATION_NODE_HPP
