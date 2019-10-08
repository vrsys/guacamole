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

#ifndef GUA_COLLISION_SHAPE_DATABASE_HPP
#define GUA_COLLISION_SHAPE_DATABASE_HPP

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/Singleton.hpp>
#include <gua/databases/Database.hpp>
#include <gua/physics/CollisionShape.hpp>

namespace gua
{
namespace physics
{
/**
 * A data base for collision shapes.
 *
 * This Database stores collision shapes that can be shared among rigid bodies.
 * It can be accessed via string identifiers.
 *
 * \ingroup gua_databases
 */
class GUA_DLL CollisionShapeDatabase : public Database<CollisionShape>, public Singleton<CollisionShapeDatabase>
{
  public:
    /**
     * Adds a collision shape to the database.
     *
     * \param name  String identifier.
     * \param shape A pointer to the collision shape.
     */
    static void add_shape(const std::string& name, CollisionShape* shape);

    friend class Singleton<CollisionShapeDatabase>;

  private:
    CollisionShapeDatabase() {}

    ~CollisionShapeDatabase() {}
};

} // namespace physics
} // namespace gua

#endif // GUA_COLLISION_SHAPE_DATABASE_HPP
