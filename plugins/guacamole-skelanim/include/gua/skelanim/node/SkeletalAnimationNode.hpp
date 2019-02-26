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
#include <gua/skelanim/utils/SkeletalAnimation.hpp>
#include <gua/skelanim/utils/Skeleton.hpp>
#include <gua/platform.hpp>

namespace gua
{
class SkinnedMeshResource;
class SkeletalAnimation;
struct Bone;

namespace node
{
/**
 * @brief represents an entity with geometry and bone hierarchy
 * which can be animated and influencs the mesh
 * @details has two current animations and two times which
 * determinate the current poses in the anims
 * the blend factor controls how much theseconds pose influecnes the first
 *
 */
class GUA_SKELANIM_DLL SkeletalAnimationNode : public GeometryNode
{
  public: // c'tor / d'tor
    inline SkeletalAnimationNode(std::string const& node_name = ""){};

    SkeletalAnimationNode(std::string const& node_name,
                          std::vector<std::string> const& geometry_description,
                          std::vector<std::shared_ptr<Material>> const& materials,
                          Skeleton const&,
                          math::mat4 const& transform = math::mat4::identity());

  public: // methods
    Node* get();

    /**
     * Get the strings referring to an entry in guacamole's GeometryDatabase.
     */
    std::vector<std::string> const& get_geometry_descriptions() const;

    /**
     * Set the path to the model file containing a skeleton.
     */
    void set_skeleton_description(std::string const& geometry_description);
    /**
     * Set the strings referring to entries in guacamole's GeometryDatabase.
     */
    void set_geometry_descriptions(std::vector<std::string> const& geometry_descriptions);

    /**
     * This is only for the multifield handling in avango
     */
    void add_material(std::shared_ptr<Material> const& material);
    /**
     * A string referring to an entry in guacamole's MaterialShaderDatabase.
     */

    std::vector<std::shared_ptr<Material>> const& get_materials() const;

    std::shared_ptr<Material> const& get_material(unsigned index) const;

    void set_material(std::shared_ptr<Material> material, unsigned index);

    inline void clear_materials() { materials_.clear(); }

    bool get_render_to_gbuffer() const;
    void set_render_to_gbuffer(bool enable);

    bool get_render_to_stencil_buffer() const;
    void set_render_to_stencil_buffer(bool enable);

    void set_bones(std::vector<Bone> const& bones);
    std::vector<Bone> const& get_bones() const;
    /**
     * Implements ray picking for a triangular mesh
     */
    void ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits) override;

    /**
     * @brief updates bounding boxes
     * @details by transforming bone boex with current bone
     * transforms and accumulating them
     */
    void update_bounding_box() const override;

    std::vector<math::BoundingBox<math::vec3>> get_bone_boxes();

    void update_cache() override;

    std::vector<std::shared_ptr<SkinnedMeshResource>> const& get_geometries() const;

    // animation related methods
    void add_animations(std::string const& file_name, std::string const& animation_name);

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

    /**
     * @brief returns current bone transformations
     * @details does not trigger an update of the transforms
     * @return current bone transforms
     */
    std::vector<scm::math::mat4f> const& get_bone_transforms() const;

    /**
     * @brief recalculates the bone transformations with the current parameters
     */
    void update_bone_transforms();

    /**
     * Accepts a visitor and calls concrete visit method.
     *
     * This method implements the visitor pattern for Nodes.
     *
     * @param visitor  A visitor to process the GeometryNode's data.
     */
    void accept(NodeVisitor& visitor) override;

  protected:
    std::shared_ptr<Node> copy() const override;

  private: // attributes e.g. special attributes for drawing
    std::vector<std::shared_ptr<SkinnedMeshResource>> geometries_;
    std::vector<std::string> geometry_descriptions_;
    bool geometry_changed_;

    std::vector<std::shared_ptr<Material>> materials_;
    bool render_to_gbuffer_;
    bool render_to_stencil_buffer_;

    // attributes related to animation
    Skeleton skeleton_;
    std::map<std::string, SkeletalAnimation> animations_;

    bool new_bones_;
    bool has_anims_;

    const static std::string none_loaded;

    float blend_factor_;
    std::string anim_1_;
    std::string anim_2_;
    float anim_time_1_;
    float anim_time_2_;

    std::vector<scm::math::mat4f> bone_transforms_;
};

} // namespace node
} // namespace gua

#endif // GUA_SKELETAL_ANIMATION_NODE_HPP
