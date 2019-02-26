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

#ifndef GUA_TRIANGLE_SHAPE_HPP
#define GUA_TRIANGLE_SHAPE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/math/math.hpp>
#include <gua/physics/CollisionShape.hpp>
#include <gua/renderer/TriMeshLoader.hpp>
#include <gua/renderer/TriMeshRessource.hpp>

// external headers
#include <memory>
#include <string>
#include <vector>
#include <btBulletDynamicsCommon.h>

namespace HACD
{
template <typename T>
class Vec3;
}

namespace gua
{
class Mesh;

namespace physics
{
/**
 * A class representing a concave triangular mesh collision shape.
 *
 * The class uses bullet's native BVH triangle mesh for static environments.
 * For dynamic rigid bodies this class uses Hierarchical Approximate Convex
 * Decomposition (HACD).
 */
class GUA_DLL TriangleMeshShape : public CollisionShape
{
  public:
    /**
     * Constructor.
     *
     * Creates an empty triangle mesh shape.
     */
    TriangleMeshShape()
        : CollisionShape(false, false, false), concave_tri_mesh_(nullptr), concave_shape_(nullptr), scaling_(math::vec3(1.f, 1.f, 1.f)), hacd_min_clusters(2), hacd_max_ch_vertices(100),
          hacd_concavity(80), hacd_add_extra_dist_points(false), hacd_add_neighbours_dist_points(false), hacd_add_faces_points(false)
    {
    }

    /**
     * Destructor.
     *
     * Deletes the triangle shape and frees all associated data.
     */
    virtual ~TriangleMeshShape();

    /**
     * Prepares a triangle mesh for static rigid bodies.
     *
     * This builds the BVH for static scenes. Please use
     * build_from_geometry_dynamic() if you need dynamical simulation of
     * the rigid body associated with this shape.
     *
     * \param geometry_list The list of geometries from the Geometry
     *                      Database.
     * \sa build_from_geometry_dynamic()
     */
    void build_from_geometry_static(const std::vector<std::string>& geometry_list);

    /**
     * Prepares a triangle mesh for dynamic rigid bodies.
     *
     * This uses HACD to divide the given meshes into convex hulls allowing
     * the TriangleMeshShape take part in the dynamical simulation.
     *
     * \param geometry_list The list of geometries from the Geometry
     *                      Database.
     * \sa build_from_geometry_static()
     */
    void build_from_geometry_dynamic(const std::vector<std::string>& geometry_list);

    /**
     * Sets local scaling to the shape.
     *
     * \param scaling Scaling values
     */
    void set_scaling(const math::vec3& scaling);

    /**
     * Gets local scaling of the shape.
     *
     * \return Current scaling
     */
    math::vec3 const& get_scaling() const { return scaling_; }

    /**
     * The factory method that creates a triangle mesh from the given
     *        geometries.
     *
     * \param geometry_list The list of geometries from the Geometry
     *                      Database.
     * \param build_static  Prepare the shape for static rigid bodies.
     * \param build_dynamic Prepare the shape for dynamic rigid bodies.
     */
    static TriangleMeshShape* FromGeometry(const std::vector<std::string>& geometry_list, bool build_static, bool build_dynamic);

    /**
     * The factory method that creates a triangle mesh from the meshes
     *        previously loaded by GeometryNode.
     *
     * \param file_name     The filename where the geometries was loaded from.
     * \param build_static  Prepare the shape for static rigid bodies.
     * \param build_dynamic Prepare the shape for dynamic rigid bodies.
     * \param flags         GeometryLoader flags.
     */
    static TriangleMeshShape* FromGeometryFile(const std::string& file_name, bool build_static, bool build_dynamic, unsigned flags = TriMeshLoader::DEFAULTS);

  private:
    virtual void construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform);

    virtual btCollisionShape* construct_static();

  private:
    /**
     * Takes an index for a vertex array if the given vertex exists.
     *        Otherwise adds the vertex and returns its index.
     *
     * This function is necessary because HACD requires a clean duplicate-free
     * vertex array.
     *
     * \param points The vertex array.
     * \param v      The vertex to search.
     */
    static long get_point_or_add(std::vector<HACD::Vec3<double>>& points, const math::vec3& v);

    /**
     * Performs HACD on the mesh. The function can optionally save
     *        the result to VRML file for debugging purposes.
     *
     * \param mesh      The mesh.
     * \param file_name File name of WRL-file to save a decomposed mesh.
     */
    void decompose_to_convex(std::shared_ptr<TriMeshRessource> const& mesh, std::string const& file_name = "");

    btTriangleMesh* concave_tri_mesh_;
    btBvhTriangleMeshShape* concave_shape_;
    btAlignedObjectArray<btConvexHullShape*> convex_shapes_;
    math::vec3 scaling_;

    // Settings
    const size_t hacd_min_clusters;
    const size_t hacd_max_ch_vertices;
    const double hacd_concavity;
    const bool hacd_add_extra_dist_points;
    const bool hacd_add_neighbours_dist_points;
    const bool hacd_add_faces_points;
};

} // namespace physics

} // namespace gua

#endif // GUA_TRIANGLE_SHAPE_HPP
