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
#include <gua/node/TexturedQuadNode.hpp>

// guacamole header
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>
#include <gua/node/RayNode.hpp>

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////

TexturedQuadNode::TexturedQuadNode() {}

////////////////////////////////////////////////////////////////////////////////

TexturedQuadNode::TexturedQuadNode(std::string const& name, Configuration const& configuration, math::mat4 const& transform) : SerializableNode(name, transform), data(configuration) {}

/* virtual */ void TexturedQuadNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////

void TexturedQuadNode::update_bounding_box() const
{
    math::BoundingBox<math::vec3> geometry_bbox(math::vec3(-0.5f * data.size().x, -0.5f * data.size().y, 0.f), math::vec3(0.5f * data.size().x, 0.5f * data.size().y, 0.f));

    bounding_box_ = transform(geometry_bbox, world_transform_);

    for(auto child : get_children())
    {
        bounding_box_.expandBy(child->get_bounding_box());
    }
}

////////////////////////////////////////////////////////////////////////////////

void TexturedQuadNode::update_cache()
{
    Node::update_cache();

    if(!TextureDatabase::instance()->contains(data.texture()))
    {
        TextureDatabase::instance()->load(data.texture());
    }
}

////////////////////////////////////////////////////////////////////////////////

void TexturedQuadNode::ray_test_impl(Ray const& ray, int options, Mask const& mask, std::set<PickResult>& hits)
{
    // first of all, check bbox
    auto box_hits(::gua::intersect(ray, bounding_box_));

    // ray did not intersect bbox -- therefore it wont intersect
    if(box_hits.first == Ray::END && box_hits.second == Ray::END)
    {
        return;
    }

    // return if only first object shall be returned and the current first hit
    // is in front of the bbox entry point and the ray does not start inside
    // the bbox
    if(options & PickResult::PICK_ONLY_FIRST_OBJECT && hits.size() > 0 && hits.begin()->distance < box_hits.first && box_hits.first != Ray::END)
    {
        return;
    }

    // bbox is intersected, but check geometry only if mask tells us to check
    if(mask.check(get_tags()))
    {
        math::mat4 world_transform(get_world_transform());
        math::mat4 ori_transform(scm::math::inverse(world_transform));

        math::vec4 ori(ray.origin_[0], ray.origin_[1], ray.origin_[2], 1.0);
        math::vec4 dir(ray.direction_[0], ray.direction_[1], ray.direction_[2], 0.0);

        ori = ori_transform * ori;
        dir = ori_transform * dir;

        Ray object_ray(ori, dir, ray.t_max_);
        auto result(intersect(object_ray, math::BoundingBox<math::vec3>(math::vec3(-0.5, -0.5, 0), math::vec3(0.5, 0.5, 0))));

        float const inf(std::numeric_limits<float>::max());

        if(result.first != Ray::END)
        {
            hits.insert(PickResult(result.first, this, ori + result.first * dir, math::vec3(inf, inf, inf), math::vec3(0.f, 0.f, 1.f), math::vec3(inf, inf, inf), ori + result.first * dir + 0.5f));
        }

        if(options & PickResult::GET_WORLD_POSITIONS)
        {
            for(auto& hit : hits)
            {
                if(hit.world_position == math::vec3(inf, inf, inf))
                {
                    auto transformed(world_transform * math::vec4(hit.position.x, hit.position.y, hit.position.z, 0.0));
                    hit.world_position = scm::math::vec3(transformed.x, transformed.y, transformed.z);
                }
            }
        }

        if(options & PickResult::GET_WORLD_NORMALS)
        {
            math::mat4 normal_matrix(scm::math::inverse(scm::math::transpose(world_transform)));
            for(auto& hit : hits)
            {
                if(hit.world_normal == math::vec3(inf, inf, inf))
                {
                    auto transformed(normal_matrix * math::vec4(hit.normal.x, hit.normal.y, hit.normal.z, 0.0));
                    hit.world_normal = scm::math::normalize(scm::math::vec3(transformed.x, transformed.y, transformed.z));
                }
            }
        }
    }

    for(auto child : get_children())
    {
        // test for intersection with each child
        child->ray_test_impl(ray, options, mask, hits);
    }
}

////////////////////////////////////////////////////////////////////////////////

math::mat4 TexturedQuadNode::get_scaled_transform() const
{
    math::mat4 scale(scm::math::make_scale(math::float_t(data.size().x), math::float_t(data.size().y), math::float_t(1)));
    return get_transform() * scale;
}

////////////////////////////////////////////////////////////////////////////////

math::mat4 TexturedQuadNode::get_scaled_world_transform() const
{
    math::mat4 scale(scm::math::make_scale(math::float_t(data.size().x), math::float_t(data.size().y), math::float_t(1)));
    return get_world_transform() * scale;
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> TexturedQuadNode::copy() const { return std::make_shared<TexturedQuadNode>(*this); }

////////////////////////////////////////////////////////////////////////////////

} // namespace node
} // namespace gua
