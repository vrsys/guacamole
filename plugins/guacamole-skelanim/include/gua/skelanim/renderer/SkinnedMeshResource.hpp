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

#ifndef GUA_SKINNED_MESH_RESSOURCE_HPP
#define GUA_SKINNED_MESH_RESSOURCE_HPP

// guacamole headers
#include <gua/skelanim/platform.hpp>
#include <gua/skelanim/utils/SkinnedMesh.hpp>
#include <gua/renderer/GeometryResource.hpp>
// #include <gua/utils/KDTree.hpp>

// external headers
#include <scm/gl_core.h>
#include <mutex>
#include <vector>

namespace gua
{
struct RenderContext;
struct SharedSkinningResource;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp/FBX mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class GUA_SKELANIM_DLL SkinnedMeshResource : public GeometryResource
{
  public:
    /**
     * Default constructor.
     *
     * Creates a new and empty Mesh.
     */
    SkinnedMeshResource();

    /**
     * Constructor from an Skinned mesh.
     *
     * Initializes the mesh from a given skinned mesh.
     *
     * @param mesh mesh to store in this resource.
     * @param build_kd_tree whether to build the kd tree (not supported).
     */
    SkinnedMeshResource(SkinnedMesh const& mesh, bool build_kd_tree);

    /**
     * Draws the Mesh.
     *
     * Draws the Mesh to the given context.
     *
     * @param context          The RenderContext to draw onto.
     */
    void draw(RenderContext& context) /*const*/;

    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits);

    unsigned int num_vertices() const;
    unsigned int num_faces() const;

    scm::math::vec3 get_vertex(unsigned int i) const;
    std::vector<unsigned int> get_face(unsigned int i) const;
    SkinnedMesh const& get_mesh() const;

    /**
     * @brief calculates the bone bounding boxes
     * @details applies the transforms to bone boxes and returns them
     *
     * @param bone_transforms transform to apply
     * @return bounding boxes
     */
    std::vector<math::BoundingBox<math::vec3>> get_bone_boxes(std::vector<scm::math::mat4f> const& bone_transforms);

    friend class SkeletalAnimationRenderer;
    friend class LightingPass;

    void upload_to(RenderContext& ctx, SharedSkinningResource& resource);

  private:
    void init_bone_boxes();

    SkinnedMesh mesh_;
    std::vector<math::BoundingBox<math::vec3>> bone_boxes_;
    // public:
    //  KDTree kd_tree_;
};

} // namespace gua

#endif // GUA_SKINNED_MESH_RESSOURCE_HPP
