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

#ifndef GUA_KD_TREE_UTILS_HPP
#define GUA_KD_TREE_UTILS_HPP

#include <gua/math/math.hpp>
#include <gua/math/BoundingBox.hpp>

#include <scm/gl_core/primitives/box.h>
#include <gua/utils/Mesh.hpp>

#include <vector>

namespace gua
{
/**
 * This helper class represents a Ray.
 *
 * It has an origin, a direction and a length.
 */
struct GUA_DLL Ray
{
    Ray();
    Ray(math::vec3 const& origin, math::vec3 const& direction, math::vec3::value_type t_max);

    Ray const intersection(math::BoundingBox<math::vec3> const& box) const;
    Ray const intersection(scm::gl::box_impl<gua::math::vec3::value_type> const& box) const { return intersection(math::BoundingBox<math::vec3>(box.min_vertex(), box.max_vertex())); }

    math::vec3 origin_;
    math::vec3 direction_;

    math::vec3::value_type t_max_;

    static const math::vec3::value_type END;
};

GUA_DLL std::pair<float, float> intersect(Ray const& ray, math::BoundingBox<math::vec3> const& box);

/**
 * This helper class represents a triangle.
 *
 * It has three vertices, a normal and a visited-flag for internal KDTree usage.
 */
struct GUA_DLL Triangle
{
    Triangle();
    Triangle(unsigned face_id);

    float intersect(Mesh const& mesh, Ray const& ray) const;

    math::vec3 get_vertex(Mesh const& mesh, unsigned vertex_id) const;
    math::vec3 get_normal(Mesh const& mesh) const;
    math::vec3 get_normal_interpolated(Mesh const& mesh, math::vec3 const& position) const;
    math::vec2 get_texture_coords_interpolated(Mesh const& mesh, math::vec3 const& position) const;

    unsigned face_id_;
    mutable unsigned visit_flag_;
};

} // namespace gua

#endif // GUA_KD_TREE_UTILS_HPP
