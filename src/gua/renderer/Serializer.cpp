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
#include <gua/renderer/SerializedNode.hpp>
#include <gua/renderer/Mesh.hpp>
#include <gua/renderer/NURBS.hpp>

#include <gua/databases/GeometryDatabase.hpp>

#include <gua/scenegraph/Node.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/scenegraph/LODNode.hpp>
#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/scenegraph/VolumeNode.hpp>
#include <gua/scenegraph/PointLightNode.hpp>
#include <gua/scenegraph/SpotLightNode.hpp>
#include <gua/scenegraph/SunLightNode.hpp>
#include <gua/scenegraph/ScreenNode.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/scenegraph/SceneGraph.hpp>

// external headers
#include <stack>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Serializer::Serializer()
    : data_(nullptr),
      current_render_mask_(""),
      current_frustum_(),
      current_center_of_interest_(),
      draw_bounding_boxes_(false),
      draw_rays_(false),
      enable_frustum_culling_(false) {}

////////////////////////////////////////////////////////////////////////////////

void Serializer::check(SerializedScene* output,
                       SceneGraph const* scene_graph,
                       std::string const& render_mask,
                       bool draw_bounding_boxes,
                       bool draw_rays,
                       bool enable_frustum_culling) {

  data_ = output;

  std::size_t mesh_count = data_->meshnodes_.size();
  std::size_t nurbs_count = data_->nurbsnodes_.size();
  std::size_t volume_count = data_->volumenodes_.size();
  std::size_t point_light_count = data_->point_lights_.size();
  std::size_t spot_light_count = data_->spot_lights_.size();
  std::size_t sun_light_count = data_->sun_lights_.size();
  std::size_t ray_count = data_->rays_.size();
  std::size_t textured_quad_count = data_->textured_quads_.size();

  data_->meshnodes_.clear();
  data_->nurbsnodes_.clear();
  data_->volumenodes_.clear();
  data_->point_lights_.clear();
  data_->spot_lights_.clear();
  data_->sun_lights_.clear();
  data_->textured_quads_.clear();

  data_->bounding_boxes_.clear();
  data_->rays_.clear();
  draw_bounding_boxes_ = draw_bounding_boxes;
  draw_rays_ = draw_rays;

  if (draw_bounding_boxes_) {
    data_->materials_.insert("gua_bounding_box");
    data_->bounding_boxes_
        .reserve(mesh_count + nurbs_count + point_light_count +
                 spot_light_count + ray_count);
  }

  if (draw_rays_) {
    data_->materials_.insert("gua_bounding_box");
    data_->rays_.reserve(ray_count);
  }

  data_->materials_.insert("gua_textured_quad");

  // assuming the number of nodes stays quite constant through time,
  // reserving the old size might save some time
  data_->meshnodes_.reserve(mesh_count);
  data_->nurbsnodes_.reserve(nurbs_count);
  data_->volumenodes_.reserve(nurbs_count);
  data_->point_lights_.reserve(point_light_count);
  data_->spot_lights_.reserve(spot_light_count);
  data_->sun_lights_.reserve(sun_light_count);
  data_->textured_quads_.reserve(textured_quad_count);

  enable_frustum_culling_ = enable_frustum_culling;

  current_render_mask_ = Mask(render_mask);
  current_frustum_ = output->frustum;
  current_center_of_interest_ = output->center_of_interest;

  scene_graph->accept(*this);
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(Node* node) {
  if (is_visible(node)) {
    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(LODNode* node) {
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

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(GeometryNode* node) {

  if (is_visible(node)) {
    if (!node->data.get_geometry().empty() && !node->data.get_material().empty()) {

      add_bbox(node);

      std::shared_ptr<Mesh> mesh_ptr = std::dynamic_pointer_cast<Mesh>(
          gua::GeometryDatabase::instance()->lookup(node->data.get_geometry()));

      if (mesh_ptr) {

        data_->meshnodes_.push_back(make_serialized_node(node->get_world_transform(), node->data));

      } else {

        std::shared_ptr<NURBS> nurbs_ptr = std::dynamic_pointer_cast<NURBS>(
            gua::GeometryDatabase::instance()->lookup(node->data.get_geometry()));

        if (nurbs_ptr) {
          data_->nurbsnodes_.push_back(make_serialized_node(node->get_world_transform(), node->data));
        }
      }
    }

    data_->materials_.insert(node->data.get_material());

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(VolumeNode* node) {

  if ( is_visible(node) ) {
    if ( !node->data.get_volume().empty() ) {
      add_bbox(node);
      data_->volumenodes_.push_back(make_serialized_node(node->get_world_transform(), node->data));
    }

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(PointLightNode* node) {

  if (is_visible(node)) {

    add_bbox(node);

    data_->point_lights_.push_back(make_serialized_node(node->get_world_transform(), node->data));

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(SpotLightNode* node) {

  if (is_visible(node)) {

    add_bbox(node);

    data_->spot_lights_
        .push_back(make_serialized_node(node->get_world_transform(), node->data));

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(SunLightNode* node) {

  if (is_visible(node)) {
    data_->sun_lights_
        .push_back(make_serialized_node(node->get_world_transform(), node->data));

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(RayNode* node) {

  if (is_visible(node)) {

    if (draw_rays_) {
      GeometryNode::Configuration config;
      config.set_geometry("gua_ray_geometry");
      config.set_material("gua_bounding_box");
      data_->rays_.push_back(make_serialized_node(node->get_world_transform(), config));
    }

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

/* virtual */ void Serializer::visit(TexturedQuadNode* node) {

  if (is_visible(node)) {

    add_bbox(node);

    data_->textured_quads_
        .push_back(make_serialized_node(node->get_scaled_world_transform(), node->data));

    visit_children(node);
  }
}

////////////////////////////////////////////////////////////////////////

bool Serializer::is_visible(Node* node) const {
  bool is_visible(true);

  if (enable_frustum_culling_) {
    auto bbox(node->get_bounding_box());
    if (bbox != math::BoundingBox<math::vec3>()) {
      is_visible = current_frustum_.is_inside(bbox);
    }
  }

  if (is_visible) {
    is_visible = current_render_mask_.check(node->get_groups());
  }

  return is_visible;
}

////////////////////////////////////////////////////////////////////////

void Serializer::add_bbox(Node* node) const {
  if (draw_bounding_boxes_) {
    auto bbox(node->get_bounding_box());
    data_->bounding_boxes_.push_back(bbox);
  }
}

////////////////////////////////////////////////////////////////////////

void Serializer::visit_children(Node* node) {
  std::for_each(node->children_.begin(), node->children_.end(), std::bind(std::mem_fn(&Node::accept), std::placeholders::_1, std::ref(*this)));
}

}  // namespace gua
