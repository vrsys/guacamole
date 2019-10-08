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
#include <gua/renderer/LodResource.hpp>

#include <gua/utils/Singleton.hpp>
#include <gua/node/PLodNode.hpp>

#include <scm/gl_core/render_device.h>
#include <scm/gl_core/buffer_objects.h>
#include <scm/gl_core/shader_objects.h>
#include <scm/gl_core/texture_objects.h>
#include <scm/gl_core/render_device/opengl/util/assert.h>
#include <scm/gl_core/data_formats.h>
#include <scm/gl_core/constants.h>

#include <boost/assign/list_of.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/node/Node.hpp>

#include <lamure/ren/ray.h>
#include <lamure/ren/controller.h>

// external headers
#include <stack>
#include <algorithm>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

LodResource::LodResource(lamure::model_t model_id, bool is_pickable, math::mat4 const& local_transform) : model_id_(model_id), is_pickable_(is_pickable), local_transform_(local_transform)
{
    //compute bounds
    auto bvh = lamure::ren::model_database::get_instance()->get_model(model_id)->get_bvh();

    scm::math::vec3f min_vertex(std::numeric_limits<float>::max());
    scm::math::vec3f max_vertex(std::numeric_limits<float>::lowest());
    for (const auto& box : bvh->get_bounding_boxes()) {
      max_vertex.x = std::max(max_vertex.x, box.max_vertex().x);
      max_vertex.y = std::max(max_vertex.y, box.max_vertex().y);
      max_vertex.z = std::max(max_vertex.z, box.max_vertex().z);
      min_vertex.x = std::min(min_vertex.x, box.min_vertex().x);
      min_vertex.y = std::min(min_vertex.y, box.min_vertex().y);
      min_vertex.z = std::min(min_vertex.z, box.min_vertex().z);
    }

    bounding_box_.min = min_vertex;
    bounding_box_.max = max_vertex;
}

////////////////////////////////////////////////////////////////////////////////

LodResource::~LodResource() {}

////////////////////////////////////////////////////////////////////////////////

void LodResource::draw(RenderContext const& ctx,
                       lamure::context_t context_id,
                       lamure::view_t view_id,
                       lamure::model_t model_id,
                       scm::gl::vertex_array_ptr const& vertex_array,
                       std::unordered_set<lamure::node_t> const& nodes_in_frustum,
                       scm::gl::primitive_topology const type,
                       scm::math::mat4d model_view_matrix,
                       bool draw_sorted) const
{
    lamure::ren::model_database* database = lamure::ren::model_database::get_instance();
    lamure::ren::cut_database* cuts = lamure::ren::cut_database::get_instance();

    lamure::ren::cut& cut = cuts->get_cut(context_id, view_id, model_id);
    std::vector<lamure::ren::cut::node_slot_aggregate>& node_list = cut.complete_set();
    lamure::ren::bvh const* bvh = database->get_model(model_id)->get_bvh();

    uint32_t primitives_per_node = database->get_primitives_per_node();
    uint32_t primitives_per_node_of_model = bvh->get_primitives_per_node();

    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(vertex_array);
    ctx.render_context->apply();

    if(draw_sorted)
    {
        // sorting BEGIN
        std::vector<lamure::ren::cut::node_slot_aggregate> node_render_list;

        node_render_list.reserve(nodes_in_frustum.size());

        for(const auto& n : node_list)
        {
            if(nodes_in_frustum.find(n.node_id_) != nodes_in_frustum.end())
            {
                node_render_list.push_back(n);
            }
        }
        auto const& bounding_box_vector = bvh->get_bounding_boxes();

        auto compute_dist = [](scm::math::vec3f const& v3, scm::math::vec4d const& v4) {
            scm::math::vec3d dist_vec(double(v3[2]) - v4[2]);

            return scm::math::length_sqr(dist_vec);
        };

        std::sort(node_render_list.begin(), node_render_list.end(), [&](lamure::ren::cut::node_slot_aggregate const& lhs, lamure::ren::cut::node_slot_aggregate const& rhs) {
            bool result = compute_dist(scm::math::vec3f(0), scm::math::vec4d(model_view_matrix * scm::math::vec4d(scm::math::vec3d(bounding_box_vector[lhs.node_id_].center()), 1.0))) <
                          compute_dist(scm::math::vec3f(0), scm::math::vec4d(model_view_matrix * scm::math::vec4d(scm::math::vec3d(bounding_box_vector[rhs.node_id_].center()), 1.0)));

            return result;
        });

        for(const auto& n : node_render_list)
        {
            // result inside vector means the node is out of frustum
            // if (nodes_in_frustum.find(n.node_id_) != nodes_in_frustum.end()) {

            ctx.render_context->draw_arrays(type, n.slot_id_ * primitives_per_node, primitives_per_node_of_model);
            //}
        }
    }
    else
    {
        for(const auto& n : node_list)
        {
            if(nodes_in_frustum.find(n.node_id_) != nodes_in_frustum.end())
            {
                ctx.render_context->draw_arrays(type, n.slot_id_ * primitives_per_node, primitives_per_node_of_model);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
math::mat4 const& LodResource::local_transform() const { return local_transform_; }

////////////////////////////////////////////////////////////////////////////////

void LodResource::ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits)
{
    if(!is_pickable_)
        return;

    bool has_hit = false;
    PickResult pick = PickResult(0.f, owner, ray.origin_, math::vec3(0.f, 0.f, 0.f), math::vec3(0.f, 1.f, 0.f), math::vec3(0.f, 1.f, 0.f), math::vec2(0.f, 0.f));

    const auto model_transform = owner->get_cached_world_transform();
    const auto world_origin = ray.origin_;
    const auto world_direction = scm::math::normalize(ray.direction_);

    lamure::ren::ray lod_ray(math::vec3f(world_origin), math::vec3f(world_direction), scm::math::length(ray.direction_));
    lamure::ren::ray::intersection intersection;

    auto lod_node = reinterpret_cast<node::PLodNode*>(owner);

    float aabb_scale = 9.0f;
    unsigned int max_depth = 255;
    unsigned int surfel_skip = 1;
    bool wysiwyg = true;

    lamure::ren::controller* controller = lamure::ren::controller::get_instance();
    lamure::model_t model_id = controller->deduce_model_id(lod_node->get_geometry_description());

    if(lod_ray.intersect_model(model_id, math::mat4f(model_transform), aabb_scale, max_depth, surfel_skip, wysiwyg, intersection))
    {
        has_hit = true;
        pick.distance = intersection.distance_;

        pick.world_position = intersection.position_;
        auto object_position = scm::math::inverse(model_transform) * gua::math::vec4(intersection.position_.x, intersection.position_.y, intersection.position_.z, 1.0);
        pick.position = math::vec3(object_position.x, object_position.y, object_position.z);
        pick.normal = intersection.normal_;
    }

    if(has_hit)
    {
        if(hits.empty())
        {
            hits.insert(pick);
        }
        else if(pick.distance < hits.begin()->distance)
        {
            hits.insert(pick);
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////

} // namespace gua
