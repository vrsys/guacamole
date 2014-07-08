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
#include <gua/renderer/PLODRessource.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/renderer/RenderContext.hpp>
#include <gua/renderer/ShaderProgram.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/node/Node.hpp>

// external headers
#include <stack>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

PLODRessource::PLODRessource(pbr::model_t model_id, bool is_pickable)
    : model_id_(model_id), is_pickable_(is_pickable) {

  scm::gl::boxf bb = pbr::ren::ModelDatabase::GetInstance()->GetModel(model_id)
      ->kdn_tree()->bounding_boxes()[0];

  bounding_box_.min = bb.min_vertex();
  bounding_box_.max = bb.max_vertex();
}

////////////////////////////////////////////////////////////////////////////////

void PLODRessource::draw(
    RenderContext const& ctx,
    pbr::context_t context_id,
    pbr::view_t view_id,
    pbr::model_t model_id,
    scm::gl::vertex_array_ptr const& vertex_array,
    std::vector<unsigned int> const& culling_results) const {

  pbr::ren::ModelDatabase* database = pbr::ren::ModelDatabase::GetInstance();
  pbr::ren::CutDatabase* cuts = pbr::ren::CutDatabase::GetInstance();

  pbr::ren::Cut& cut = cuts->GetCut(context_id, view_id, model_id);
  std::vector<pbr::ren::Cut::NodeSlotAggregate>& node_list = cut.complete_set();
  pbr::ren::KdnTree const* kdn_tree = database->GetModel(model_id)->kdn_tree();

  uint32_t surfels_per_node = database->surfels_per_node();
  uint32_t surfels_per_node_of_model = kdn_tree->surfels_per_node();

  scm::gl::context_vertex_input_guard vig(ctx.render_context);

  ctx.render_context->bind_vertex_array(vertex_array);
  ctx.render_context->apply();

  pbr::node_t node_counter = 0;
  for (const auto& n : node_list) {
    ++node_counter;
    // 0 = completely inside of frustum,
    // 1 = completely outside of frustum,
    // 2 = intersects frustum
    if (culling_results[node_counter] != 1) {
      ctx.render_context->draw_arrays(scm::gl::PRIMITIVE_POINT_LIST,
                                      n.slot_id_ * surfels_per_node,
                                      surfels_per_node_of_model);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

void PLODRessource::ray_test(Ray const& ray,
                             PickResult::Options options,
                             node::Node* owner,
                             std::set<PickResult>& hits) {

  if (!is_pickable_)
    return;

  const auto* tree =
      pbr::ren::ModelDatabase::GetInstance()->GetModel(model_id_)->kdn_tree();
  const uint32_t num_nodes = tree->num_nodes();
  const uint32_t fan_factor = tree->fan_factor();
  const uint32_t first_leaf = num_nodes - pow(fan_factor, tree->depth());

  const auto owner_transform = owner->get_world_transform();
  const auto world_origin = owner_transform * ray.origin_;

  bool has_hit = false;
  PickResult pick = PickResult(0.f,
                               owner,
                               ray.origin_,
                               math::vec3(0.f, 0.f, 0.f),
                               math::vec3(0.f, 1.f, 0.f),
                               math::vec3(0.f, 1.f, 0.f),
                               math::vec2(0.f, 0.f));

  std::stack<pbr::node_t> candidates;

  // there is always an intersection with root node
  candidates.push(0);

  while (!candidates.empty()) {
    pbr::node_t current = candidates.top();
    candidates.pop();

    for (pbr::node_t k = 0; k < fan_factor; ++k) {
      pbr::node_t n = tree->GetChildId(current, k);
      if (n >= num_nodes)
        break;
      Ray hit_ray(ray.intersection(tree->bounding_boxes()[n]));
      if (hit_ray.t_max_ >= 0.f) {
        if (n >= first_leaf) {
          // leaf with intersecion, check if frontmost
          float dist = scm::math::length(owner_transform * hit_ray.origin_ -
                                         world_origin);
          if (!has_hit || dist < pick.distance) {
            pick.distance = dist;
            pick.position = hit_ray.origin_;
            has_hit = true;
          }
        } else {
          // add to stack if not leaf
          candidates.push(n);
        }
      }
    }
  }
  if (has_hit)
    hits.insert(pick);
}

/////////////////////////////////////////////////////////////////////////////////

}
