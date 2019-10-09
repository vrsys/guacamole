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

// class header
#include <gua/renderer/Frustum.hpp>
#include <gua/node/RayNode.hpp>

#include <iostream>

namespace gua
{
Frustum::Frustum()
    : camera_transform_(math::mat4::identity()), screen_transform_(math::mat4::identity()), projection_(math::mat4::identity()), view_(math::mat4::identity()), planes_(6), clip_near_(0), clip_far_(0)
{
}

////////////////////////////////////////////////////////////////////////////////

Frustum Frustum::perspective(math::mat4 const& camera_transform, math::mat4 const& screen_transform, math::mat4::value_type clip_near, math::mat4::value_type clip_far)
{
    auto projection = math::compute_perspective_frustum(camera_transform.column(3), screen_transform, clip_near, clip_far);

    Frustum result;

    result.projection_ = projection;
    result.clip_near_ = clip_near;
    result.clip_far_ = clip_far;

    init_frustum_members(camera_transform, screen_transform, result);

    return result;
}

////////////////////////////////////////////////////////////////////////////////

Frustum Frustum::orthographic(math::mat4 const& camera_transform, math::mat4 const& screen_transform, math::mat4::value_type clip_near, math::mat4::value_type clip_far)
{
    auto projection = math::compute_orthographic_frustum(camera_transform.column(3), screen_transform, clip_near, clip_far);

    Frustum result;

    result.projection_ = projection;
    result.clip_near_ = clip_near;
    result.clip_far_ = clip_far;

    init_frustum_members(camera_transform, screen_transform, result);

    return result;
}

////////////////////////////////////////////////////////////////////////////////

std::vector<math::vec3> Frustum::get_corners() const
{
    std::vector<math::vec4> tmp(8);
    std::vector<math::vec3> result(8);

    auto inverse_transform(scm::math::inverse(projection_ * view_));

    tmp[0] = inverse_transform * math::vec4(-1, -1, -1, 1);
    tmp[1] = inverse_transform * math::vec4(-1, -1, 1, 1);
    tmp[2] = inverse_transform * math::vec4(-1, 1, -1, 1);
    tmp[3] = inverse_transform * math::vec4(-1, 1, 1, 1);
    tmp[4] = inverse_transform * math::vec4(1, -1, -1, 1);
    tmp[5] = inverse_transform * math::vec4(1, -1, 1, 1);
    tmp[6] = inverse_transform * math::vec4(1, 1, -1, 1);
    tmp[7] = inverse_transform * math::vec4(1, 1, 1, 1);

    for(int i(0); i < 8; ++i)
    {
        result[i] = tmp[i] / tmp[i][3];
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

bool Frustum::intersects(math::BoundingBox<math::vec3> const& bbox, std::vector<math::vec4> const& global_planes) const
{
    auto outside = [](math::vec4 const& plane, math::vec3 const& point) { return plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3] < 0; };

    for(unsigned i(0); i < 6; ++i)
    {
        auto p(bbox.min);
        if(planes_[i][0] >= 0)
            p[0] = bbox.max[0];
        if(planes_[i][1] >= 0)
            p[1] = bbox.max[1];
        if(planes_[i][2] >= 0)
            p[2] = bbox.max[2];

        // is the positive vertex outside?
        if(outside(planes_[i], p))
        {
            return false;
        }
    }

    for(auto const& plane : global_planes)
    {
        auto p(bbox.min);
        if(plane[0] >= 0)
            p[0] = bbox.max[0];
        if(plane[1] >= 0)
            p[1] = bbox.max[1];
        if(plane[2] >= 0)
            p[2] = bbox.max[2];

        // is the positive vertex outside?
        if(outside(plane, p))
        {
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////

bool Frustum::contains(math::vec3 const& point) const
{
    auto outside = [](math::vec4 const& plane, math::vec3 const& point) { return plane[0] * point[0] + plane[1] * point[1] + plane[2] * point[2] + plane[3] < 0; };

    for(unsigned i(0); i < 6; ++i)
    {
        if(outside(planes_[i], point))
        {
            return false;
        }
    }

    return true;
}

////////////////////////////////////////////////////////////////////////////////

std::set<PickResult> const Frustum::ray_test(node::RayNode const& ray, int options) { return ray_test(ray.get_world_ray(), options); }

////////////////////////////////////////////////////////////////////////////////

// adapted from http://tog.acm.org/resources/GraphicsGems/gemsii/RayCPhdron.c

std::set<PickResult> const Frustum::ray_test(Ray const& ray, int options)
{
    std::set<PickResult> result;

    math::vec3::value_type tnear = 0;
    math::vec3::value_type tfar = ray.t_max_;

    int fnorm_num(0);
    int bnorm_num(0);

    // Test each plane in polyhedron
    // for ( pln = &phdrn[ph_num-1] ; ph_num-- ; pln-- ) {
    for(unsigned i(0); i < 6; ++i)
    {
        // Compute intersection point T and sidedness
        math::vec3::value_type vd = ray.direction_.x * planes_[i].x + ray.direction_.y * planes_[i].y + ray.direction_.z * planes_[i].z;
        math::vec3::value_type vn = ray.origin_.x * planes_[i].x + ray.origin_.y * planes_[i].y + ray.origin_.z * planes_[i].z + planes_[i].w;
        if(vd == 0.0)
        {
            // ray is parallel to plane - check if ray origin is inside plane half-space
            if(vn > 0.0)
            {
                // ray origin is outside half-space
                return result;
            }
        }
        else
        {
            // ray not parallel - get distance to plane
            math::vec3::value_type t = -vn / vd;
            if(vd > 0.0)
            {
                if(t > tfar)
                {
                    // front face - T is a near point
                    return result;
                }

                if(t > tnear)
                {
                    // hit near face, update normal
                    fnorm_num = i;
                    tnear = t;
                }
            }
            else
            {
                if(t < tnear)
                {
                    // back face - T is a far point
                    return result;
                }
                if(t < tfar)
                {
                    // hit far face, update normal
                    bnorm_num = i;
                    tfar = t;
                }
            }
        }
    }

    // survived all tests
    if(tnear >= 0.0)
    {
        // outside, hitting front face
        math::vec3 pos(ray.origin_ + tnear * ray.direction_);
        math::vec3 normal(planes_[fnorm_num].x, planes_[fnorm_num].y, planes_[fnorm_num].z);
        result.insert(PickResult(tnear, nullptr, pos, pos, normal, normal, math::vec2(0, 0)));
    }

    if(tfar < ray.t_max_)
    {
        // inside, hitting back face
        math::vec3 pos(ray.origin_ + tfar * ray.direction_);
        math::vec3 normal(planes_[bnorm_num].x, planes_[bnorm_num].y, planes_[bnorm_num].z);
        result.insert(PickResult(tfar, nullptr, pos, pos, normal, normal, math::vec2(0, 0)));
    }

    return result;
}

////////////////////////////////////////////////////////////////////////////////

void Frustum::init_frustum_members(math::mat4 const& camera_transform, math::mat4 const& screen_transform, Frustum& frustum)
{
    math::mat4 view_transform(screen_transform);
    view_transform[12] = 0.f;
    view_transform[13] = 0.f;
    view_transform[14] = 0.f;
    view_transform[15] = 1.f;

    frustum.camera_transform_ = camera_transform;
    frustum.screen_transform_ = screen_transform;

    view_transform = scm::math::make_translation(frustum.get_camera_position()) * view_transform;

    frustum.view_ = scm::math::inverse(view_transform);

    auto projection_view(frustum.projection_ * frustum.view_);

    // store normals

    // left plane
    frustum.planes_[0] =
        math::vec4(projection_view[3] + projection_view[0], projection_view[7] + projection_view[4], projection_view[11] + projection_view[8], projection_view[15] + projection_view[12]);

    // right plane
    frustum.planes_[1] =
        math::vec4(projection_view[3] - projection_view[0], projection_view[7] - projection_view[4], projection_view[11] - projection_view[8], projection_view[15] - projection_view[12]);

    // bottom plane
    frustum.planes_[2] =
        math::vec4(projection_view[3] + projection_view[1], projection_view[7] + projection_view[5], projection_view[11] + projection_view[9], projection_view[15] + projection_view[13]);

    // top plane
    frustum.planes_[3] =
        math::vec4(projection_view[3] - projection_view[1], projection_view[7] - projection_view[5], projection_view[11] - projection_view[9], projection_view[15] - projection_view[13]);

    // near plane
    frustum.planes_[4] =
        math::vec4(projection_view[3] + projection_view[2], projection_view[7] + projection_view[6], projection_view[11] + projection_view[10], projection_view[15] + projection_view[14]);

    // far plane
    frustum.planes_[5] =
        math::vec4(projection_view[3] - projection_view[2], projection_view[7] - projection_view[6], projection_view[11] - projection_view[10], projection_view[15] - projection_view[14]);
}

} // namespace gua
