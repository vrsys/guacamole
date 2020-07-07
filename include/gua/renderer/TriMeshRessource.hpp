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

#ifndef GUA_TRIMESH_RESSOURCE_HPP
#define GUA_TRIMESH_RESSOURCE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/GeometryResource.hpp>
#include <gua/utils/Mesh.hpp>
#include <gua/utils/KDTree.hpp>

// external headers
#include <scm/gl_core.h>

#include <mutex>
#include <thread>

#include <vector>

namespace gua
{
struct RenderContext;

/**
 * Stores geometry data.
 *
 * A mesh can be loaded from an Assimp mesh and the draw onto multiple
 * contexts.
 * Do not use this class directly, it is just used by the Geometry class to
 * store the individual meshes of a file.
 */
class GUA_DLL TriMeshRessource : public GeometryResource
{
  public: // typedefs, enums
    enum Flags
    {
        DEFAULTS = 0,
        SAVE_TANGENTS = 1 << 0,
        SAVE_BITANGENTS = 1 << 1
    };

  public:
    /**
     * Default constructor.
     *
     * Creates a new and empty Mesh.
     */
    TriMeshRessource();

    /**
     * Constructor from an Assimp mesh.
     *
     * Initializes the mesh from a given Assimp mesh.
     *
     * \param mesh             The Assimp mesh to load the data from.
     */
    TriMeshRessource(Mesh const& mesh, bool build_kd_tree);

    /**
     * Draws the Mesh.
     *
     * Draws the Mesh to the given context.
     *
     * \param context          The RenderContext to draw onto.
     */
    void draw(RenderContext& context) const;

    /**
     * Draws Instances of Mesh.
     *
     * Draws the Mesh Instances to the given context.
     *
     * \param context          The RenderContext to draw onto.
     * \param instance_count   The Number of Mesh Instances to draw.
     */
    void draw_instanced(RenderContext& ctx, int instance_count, int base_vertex = 0, int base_instance = 0) const;

    void draw_instanced_partially(RenderContext& ctx, int instance_count, int start_index, int num_indices) const;

    void bind_buffers_unsafe(RenderContext& ctx) const;

    void draw_instanced_partially_unsafe(RenderContext& ctx, int instance_count, int start_index, int num_indices) const;

   /**
     * Draws the kD-tree of the Mesh.
     *
     * Draws the kD-tree Mesh to the given context.
     *
     * \param context          The RenderContext to draw onto.
     */
    void draw_kdtree(RenderContext& context) const;

    void ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits) override;

    inline unsigned int num_vertices() const { return mesh_.num_vertices; }
    inline unsigned int num_faces() const { return mesh_.num_triangles; }

    math::vec3 get_vertex(unsigned int i) const;
    std::vector<unsigned int> get_face(unsigned int i) const;

    bool save_to_binary(std::string const& input_filename, std::string const& output_filename, const char* filename_gua_kdtree, unsigned flags = DEFAULTS);

    void set_original_material_name(std::string const& in_material_name);
    std::string get_original_material_name() const;
    
    void set_base_filename(std::string const& in_base_filename);
    std::string get_base_filename() const;

  private:
    void upload_to(RenderContext& context) const;
    void upload_kdtree_to(RenderContext& context) const;
    
    KDTree kd_tree_;
    Mesh mesh_;

    std::string original_material_name_;
    std::string base_filename_;
};

} // namespace gua

#endif // GUA_TRIMESH_RESSOURCE_HPP
