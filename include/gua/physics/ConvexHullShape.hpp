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

#ifndef GUA_CONVEX_HULL_SHAPE_HPP
#define GUA_CONVEX_HULL_SHAPE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/physics/CollisionShape.hpp>
#include <gua/physics/PhysicsUtils.hpp>
#include <gua/renderer/TriMeshLoader.hpp>

class btConvexHullShape;

namespace gua
{
namespace physics
{
/**
 * A class representing a convex hull collision shape.
 *
 * This class implements an implicit convex hull of an array of vertices.
 * The convex hull shape can be used for both static and dynamic rigid bodies.
 */
class GUA_DLL ConvexHullShape : public CollisionShape
{
  public:
    /**
     * Constructor.
     *
     * Creates an empty convex hull shape.
     */
    ConvexHullShape();

    /**
     * Destructor.
     *
     * Deletes the convex hull shape and frees all associated data.
     */
    virtual ~ConvexHullShape();

    /**
     * Adds a point to the convex hull.
     *
     * This method allows to build a convex hull by adding one point at a time.
     *
     * \param point The point.
     */
    void add_point(const math::vec3& point);

    /**
     * Creates a convex hull from the given geometry.
     *
     * \param geometry_list               The list of geometries from the
     *                                    Geometry Database.
     * \param compensate_collision_margin If true, the shape will be scaled
     *                                    down a little bit to compensate the
     *                                    collision margin.
     */
    void build_from_geometry(const std::vector<std::string>& geometry_list, bool compensate_collision_margin = true);

    /**
     * The factory method that creates a convex hull from the given
     *        geometry.
     *
     * \param geometry_list               The list of geometries from the
     *                                    Geometry Database.
     * \param compensate_collision_margin If true, the shape will be scaled
     *                                    down a little bit to compensate the
     *                                    collision margin.
     */
    static ConvexHullShape* FromGeometry(const std::vector<std::string>& geometry_list, bool compensate_collision_margin = true);

    /**
     * The factory method that creates a convex hull from the meshes
     *        previously loaded by GeometryNode.
     *
     * \param file_name     The filename where the geometries was loaded from.
     * \param compensate_collision_margin If true, the shape will be scaled
     *                                    down a little bit to compensate the
     *                                    collision margin.
     * \param flags         GeometryLoader flags.
     */
    static ConvexHullShape* FromGeometryFile(const std::string& file_name, bool compensate_collision_margin = true, unsigned flags = TriMeshLoader::DEFAULTS);

  private:
    virtual void construct_dynamic(btCompoundShape* bullet_shape, const btTransform& base_transform);

    virtual btCollisionShape* construct_static();

    btConvexHullShape* shape_;
};

} // namespace physics
} // namespace gua

#endif // GUA_CONVEX_HULL_SHAPE_HPP
