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
#include <gua/renderer/Serializer.hpp>

// guacamole headers
#include <gua/platform.hpp>

#include <gua/databases/GeometryDatabase.hpp>

#include <gua/node/Node.hpp>
#include <gua/node/TransformNode.hpp>
#include <gua/node/LODNode.hpp>
#include <gua/node/SerializableNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

// external headers
#include <stack>
#include <utility>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Serializer::Serializer()
    : data_(nullptr),
      current_frustum_(),
      current_center_of_interest_(),
      enable_frustum_culling_(false) {}

////////////////////////////////////////////////////////////////////////////////

void Serializer::check(SerializedScene& output,
                       SceneGraph const& scene_graph,
                       Mask const& mask,
                       bool enable_frustum_culling) {

  data_ = &output;
  data_->nodes.clear();
  data_->bounding_boxes.clear();
  data_->clipping_plane_centers.clear();
  data_->clipping_plane_normals.clear();

  enable_frustum_culling_     = enable_frustum_culling;
  current_render_mask_        = mask;
  current_frustum_            = output.frustum;
  current_center_of_interest_ = output.center_of_interest;

  for (auto plane: scene_graph.get_clipping_plane_nodes()) {
    data_->clipping_plane_centers.push_back(plane->get_center());
    data_->clipping_plane_normals.push_back(plane->get_normal());
  }

  scene_graph.accept(*this);
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(node::Node* node) {
  if (is_visible(node)) {
    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(node::LODNode* node) {
  if (is_visible(node)) {

    float distance_to_camera(scm::math::length(node->get_world_position() - current_center_of_interest_));

    unsigned child_index(0);

    if (!node->data.get_lod_distances().empty()) {

      child_index = node->get_children().size();

      for (unsigned i(0); i < node->data.get_lod_distances().size(); ++i) {
        if (node->data.get_lod_distances()[i] > distance_to_camera) {
          child_index = i;
          break;
        }
      }
    }

    if (child_index < node->get_children().size()) {
      node->get_children()[child_index]->accept(*this);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(node::SerializableNode* node) {
  if (is_visible(node)) {
    data_->nodes[std::type_index(typeid(*node))].push_back(node);

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////////////

bool Serializer::is_visible(node::Node* node) const {
  bool is_visible(true);


  // check whether bounding box is (partially) within frustum
  if (enable_frustum_culling_) {
    auto bbox(node->get_bounding_box());
    if (bbox != math::BoundingBox<math::vec3>()) {
      is_visible = current_frustum_.intersects(bbox);
    }
  }

  // check whether at least one point of the bounding box isn't beyond all
  // clipping planes
  if (is_visible) {
    is_visible = check_clipping_planes(node);
  }


  // check whether mask allows rendering
  if (is_visible) {
    is_visible = current_render_mask_.check(node->get_tags());
  }

  if (is_visible && node->get_draw_bounding_box()) {
    data_->bounding_boxes.push_back(node->get_bounding_box());
  }

  return is_visible;
}

////////////////////////////////////////////////////////////////////////////////

bool Serializer::check_clipping_planes(node::Node* node) const {
  if (!data_->clipping_plane_centers.empty()) {
    auto bbox(node->get_bounding_box());
    std::vector<math::vec4f> corners(
    {
      math::vec4f(bbox.min.x, bbox.min.y, bbox.min.z, 1),
      math::vec4f(bbox.max.x, bbox.min.y, bbox.min.z, 1),
      math::vec4f(bbox.max.x, bbox.max.y, bbox.min.z, 1),
      math::vec4f(bbox.min.x, bbox.max.y, bbox.min.z, 1),
      math::vec4f(bbox.min.x, bbox.max.y, bbox.max.z, 1),
      math::vec4f(bbox.min.x, bbox.min.y, bbox.max.z, 1),
      math::vec4f(bbox.max.x, bbox.min.y, bbox.max.z, 1),
      math::vec4f(bbox.max.x, bbox.max.y, bbox.max.z, 1)
    }
    );

    for (auto corner: corners) {
      bool corner_visible(true);

      for (unsigned i(0); i < data_->clipping_plane_centers.size(); ++i) {
        auto to_center(scm::math::normalize(corner - data_->clipping_plane_centers[i]));

        if (scm::math::dot(to_center, data_->clipping_plane_normals[i]) > 0) {
          corner_visible = false;
          break;
        }
      }

      if (corner_visible) {
        return true;
      }
    }

    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////

void Serializer::visit_children(node::Node* node) {
  for (auto & c : node->children_) { c->accept(*this); }
}



}  // namespace gua
