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
#include <gua/renderer/PLODResource.hpp>

#include <gua/utils/Singleton.hpp>
#include <gua/node/PLODNode.hpp>

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

#include <pbr/ren/ray.h>
#include <pbr/ren/controller.h>

// external headers
#include <stack>
#include <algorithm>

namespace gua
{
////////////////////////////////////////////////////////////////////////////////

PLODResource::PLODResource(pbr::model_t model_id, bool is_pickable, math::mat4 const& local_transform) : model_id_(model_id), is_pickable_(is_pickable), local_transform_(local_transform)
{
    scm::gl::boxf bb = pbr::ren::ModelDatabase::GetInstance()->GetModel(model_id)->bvh()->bounding_boxes()[0];

    bounding_box_.min = bb.min_vertex();
    bounding_box_.max = bb.max_vertex();
}

////////////////////////////////////////////////////////////////////////////////

PLODResource::~PLODResource() {}

////////////////////////////////////////////////////////////////////////////////

void PLODResource::draw(RenderContext const& ctx,
                        pbr::context_t context_id,
                        pbr::view_t view_id,
                        pbr::model_t model_id,
                        scm::gl::vertex_array_ptr const& vertex_array,
                        std::unordered_set<pbr::node_t> const& nodes_in_frustum) const
{
    pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
    pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();

    pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
    std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();
    pbr::ren::Bvh const* bvh = database->GetModel(model_id)->bvh();

    uint32_t surfels_per_node = database->surfels_per_node();
    uint32_t surfels_per_node_of_model = bvh->surfels_per_node();

    scm::gl::context_vertex_input_guard vig(ctx.render_context);

    ctx.render_context->bind_vertex_array(vertex_array);
    ctx.render_context->apply();

    for(const auto& n : node_list)
    {
        // result inside vector means the node is out of frustum
        if(nodes_in_frustum.find(n.node_id_) != nodes_in_frustum.end())
        {
            ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST, n.slot_id_ * surfels_per_node, surfels_per_node_of_model);
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
math::mat4 const& PLODResource::local_transform() const { return local_transform_; }

////////////////////////////////////////////////////////////////////////////////

void PLODResource::ray_test(Ray const& ray, int options, node::Node* owner, std::set<PickResult>& hits)
{
    if(!is_pickable_)
        return;

    bool has_hit = false;
    PickResult pick = PickResult(0.f, owner, ray.origin_, math::vec3(0.f, 0.f, 0.f), math::vec3(0.f, 1.f, 0.f), math::vec3(0.f, 1.f, 0.f), math::vec2(0.f, 0.f));

    const auto model_transform = owner->get_cached_world_transform();
    const auto world_origin = ray.origin_;
    const auto world_direction = scm::math::normalize(ray.direction_);

    pbr::ren::Ray plod_ray(math::vec3f(world_origin), math::vec3f(world_direction), scm::math::length(ray.direction_));
    pbr::ren::Ray::Intersection intersection;

    auto plod_node = reinterpret_cast<node::PLODNode*>(owner);

    float aabb_scale = 9.0f;
    unsigned int max_depth = 255;
    unsigned int surfel_skip = 1;
    bool wysiwyg = true;

    pbr::ren::Controller* controller = pbr::ren::Controller::GetInstance();
    pbr::model_t model_id = controller->DeduceModelId(plod_node->get_geometry_description());

    if(plod_ray.IntersectModel(model_id, math::mat4f(model_transform), aabb_scale, max_depth, surfel_skip, wysiwyg, intersection))
    {
        has_hit = true;
        pick.distance = intersection.distance_;
        pick.world_position = intersection.position_;
        auto object_position = scm::math::inverse(model_transform) * gua::math::vec4(intersection.position_.x, intersection.position_.y, intersection.position_.z, 1.0);
        pick.position = math::vec3(object_position.x, object_position.y, object_position.z);
        pick.normal = intersection.normal_;
    }

    if(has_hit && (hits.empty() || pick.distance < hits.begin()->distance))
    {
        hits.insert(pick);
    }
}

/////////////////////////////////////////////////////////////////////////////////

} // namespace gua
