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

#include <gua/utils/KDTree.hpp>

#include <iostream>
#include <algorithm>

namespace gua
{
namespace
{
// helper function to access mesh attributes
// no bounds checking!
// local to this translation unit
inline math::vec3 mesh_vertex(Mesh const& mesh, unsigned face, unsigned i)
{
    return math::vec3(mesh.positions[mesh.indices[face * 3 + i]].x, mesh.positions[mesh.indices[face * 3 + i]].y, mesh.positions[mesh.indices[face * 3 + i]].z);
}

inline math::vec3 mesh_normal(Mesh const& mesh, unsigned face, unsigned i)
{
    return math::vec3(mesh.normals[mesh.indices[face * 3 + i]].x, mesh.normals[mesh.indices[face * 3 + i]].y, mesh.normals[mesh.indices[face * 3 + i]].z);
}

inline math::vec2 mesh_texcoord(Mesh const& mesh, unsigned face, unsigned i) { return math::vec2(mesh.texCoords[mesh.indices[face * 3 + i]].x, mesh.texCoords[mesh.indices[face * 3 + i]].y); }

} // namespace

// Ray -------------------------------------------------------------------------

const math::vec3::value_type Ray::END(std::numeric_limits<math::vec3::value_type>::max());

Ray::Ray() : origin_(), direction_(), t_max_(-1.0) {}

Ray::Ray(math::vec3 const& origin, math::vec3 const& direction, math::vec3::value_type t_max) : origin_(origin), direction_(direction), t_max_(t_max) {}

std::pair<float, float> intersect(Ray const& ray, math::BoundingBox<math::vec3> const& box)
{
    math::vec3 t1((box.min - ray.origin_) / ray.direction_);
    math::vec3 t2((box.max - ray.origin_) / ray.direction_);

    math::vec3 tmin1(std::min(t1[0], t2[0]), std::min(t1[1], t2[1]), std::min(t1[2], t2[2]));
    math::vec3 tmax1(std::max(t1[0], t2[0]), std::max(t1[1], t2[1]), std::max(t1[2], t2[2]));

    auto tmin = std::max(std::max(tmin1[0], tmin1[1]), tmin1[2]);
    auto tmax = std::min(std::min(tmax1[0], tmax1[1]), tmax1[2]);

    if(tmax >= tmin)
    {
        if(tmin > 0.0 && tmax < ray.t_max_)
        { // there are two intersections
            return std::make_pair(tmin, tmax);
        }
        else if(tmin > 0.0)
        {
            // there is only one intersection, the ray ends inside the box
            return std::make_pair(tmin, Ray::END);
        }
        else
        { // there is only one intersection, the ray starts inside the box
            // return std::make_pair(Ray::END, tmax);
            return std::make_pair(0.0, tmax);
        }
    }

    // there is no intersection

    return std::make_pair(Ray::END, Ray::END);
}

Ray const Ray::intersection(math::BoundingBox<math::vec3> const& box) const
{
    auto hits = intersect(*this, box);

    // there are two hits -> clamp ray on both sides
    if(hits.first != END && hits.second != END)
    {
        return Ray(origin_ + direction_ * hits.first, direction_, hits.second - hits.first);
    }

    // the ray ends inside the box -> clamp the origin
    if(hits.first != END)
    {
        return Ray(origin_ + direction_ * hits.first, direction_, t_max_ - hits.first);
    }

    // the ray starts inside the box -> clamp the end
    if(hits.second != END)
    {
        return Ray(origin_, direction_, hits.second);
    }

    // there is no intersection
    return Ray();
}

// Triangle --------------------------------------------------------------------

Triangle::Triangle() : face_id_(0) {}

Triangle::Triangle(unsigned face_id) : face_id_(face_id) {}

float Triangle::intersect(Mesh const& mesh, Ray const& ray) const
{
    std::vector<math::vec3> points(3);
    // math::vec3 normal(0, 0, 0);
    for(unsigned i = 0; i < 3; ++i)
    {
        points[i] = mesh_vertex(mesh, face_id_, i);
    }

    // Find Triangle Normal
    math::vec3 normal = scm::math::cross(points[1] - points[0], points[2] - points[0]);
    scm::math::normalize(normal);

    // Find distance from LP1 and LP2 to the plane defined by the triangle
    float dist1 = scm::math::dot(ray.origin_ - points[0], normal);
    float dist2 = scm::math::dot(ray.origin_ + ray.direction_ - points[0], normal);

    if((dist1 * dist2) >= 0.0f)
    {
        return Ray::END;
    } // line doesn't cross the triangle.

    if(dist1 == dist2)
    {
        return Ray::END;
    } // line and plane are parallel

    // Find point on the line that intersects with the plane
    float t = -dist1 / (dist2 - dist1);

    if(t > ray.t_max_)
    {
        return Ray::END;
    } // intersection is too far away

    math::vec3 intersection = ray.origin_ + (ray.direction_) * t;

    // Find if the interesection point lies inside the triangle by testing it
    // against all edges
    math::vec3 test = scm::math::cross(normal, points[1] - points[0]);
    if(scm::math::dot(test, intersection - points[0]) < 0.0f)
    {
        return Ray::END;
    }

    test = scm::math::cross(normal, points[2] - points[1]);
    if(scm::math::dot(test, intersection - points[1]) < 0.0f)
    {
        return Ray::END;
    }

    test = scm::math::cross(normal, points[0] - points[2]);
    if(scm::math::dot(test, intersection - points[0]) < 0.0f)
    {
        return Ray::END;
    }

    return t;
}

math::vec3 Triangle::get_vertex(Mesh const& mesh, unsigned vertex_id) const
{
    math::vec3 vertex;
    if(vertex_id < 3)
    {
        vertex = mesh_vertex(mesh, face_id_, vertex_id);
    }

    return vertex;
}

math::vec3 Triangle::get_normal(Mesh const& mesh) const
{
    math::vec3 normal(0, 0, 0);

    for(unsigned i = 0; i < 3; ++i)
    {
        normal += mesh_normal(mesh, face_id_, i);
    }

    normal = scm::math::normalize(normal);

    return normal;
}

math::vec3 Triangle::get_normal_interpolated(Mesh const& mesh, math::vec3 const& position) const
{
    math::vec3 normal(0, 0, 0);

    normal = math::interpolate(position,
                               std::make_pair(mesh_vertex(mesh, face_id_, 0), mesh_normal(mesh, face_id_, 0)),
                               std::make_pair(mesh_vertex(mesh, face_id_, 1), mesh_normal(mesh, face_id_, 1)),
                               std::make_pair(mesh_vertex(mesh, face_id_, 2), mesh_normal(mesh, face_id_, 2)));
    normal = scm::math::normalize(normal);

    return normal;
}

math::vec2 Triangle::get_texture_coords_interpolated(Mesh const& mesh, math::vec3 const& position) const
{
    math::vec2 tex_coords(0, 0);

    tex_coords = math::interpolate(position,
                                   std::make_pair(mesh_vertex(mesh, face_id_, 0), mesh_texcoord(mesh, face_id_, 0)),
                                   std::make_pair(mesh_vertex(mesh, face_id_, 1), mesh_texcoord(mesh, face_id_, 1)),
                                   std::make_pair(mesh_vertex(mesh, face_id_, 2), mesh_texcoord(mesh, face_id_, 2)));

    return tex_coords;
}

} // namespace gua
