#/******************************************************************************
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
#include "gua/node/PLODNode.hpp"

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialShaderDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/PLODLoader.hpp>
#include <gua/renderer/PLODResource.hpp>
#include <gua/renderer/Material.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

#include <pbr/ren/policy.h>

// guacamole headers

namespace gua {
namespace node {

////////////////////////////////////////////////////////////////////////////////
PLODNode::PLODNode(std::string const& name,
                   std::string const& geometry_description,
                   std::string const& geometry_file_path,
                   std::shared_ptr<Material> const& material,
                   math::mat4 const& transform,
                   float const importance,
                   float const threshold,
                   bool const enable_backface_culling_by_normal)
    : GeometryNode(name, transform),
      geometry_(nullptr),
      geometry_changed_(true),
      geometry_description_(geometry_description),
      geometry_file_path_(geometry_file_path),
      material_(material),
      radius_scale_(importance),
      error_threshold_(threshold),
      enable_backface_culling_by_normal_(enable_backface_culling_by_normal)
    {}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<PLODResource> const& PLODNode::get_geometry() const {
  return geometry_;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& PLODNode::get_geometry_file_path() const {
  return geometry_file_path_;
}

////////////////////////////////////////////////////////////////////////////////
std::string const& PLODNode::get_geometry_description() const {
  return geometry_description_;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::set_geometry_description(std::string const& v) {
  geometry_description_ = v;
  geometry_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Material> const& PLODNode::get_material() const {
  return material_;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::set_material(std::shared_ptr<Material> const& material) {
  material_ = material;
  material_changed_ = self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::set_radius_scale(float const scale) {
  radius_scale_ = scale;
  self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
float PLODNode::get_radius_scale() {
  return radius_scale_;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::set_error_threshold(float const threshold) {
  error_threshold_ = threshold;
  self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
float PLODNode::get_error_threshold() {
  return error_threshold_;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::set_enable_backface_culling_by_normal(bool const enable_backface_culling) {
  enable_backface_culling_by_normal_ = enable_backface_culling;
  self_dirty_ = true;
}

////////////////////////////////////////////////////////////////////////////////
bool PLODNode::get_enable_backface_culling_by_normal() {
  return enable_backface_culling_by_normal_;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::ray_test_impl(Ray const& ray,
                             int options,
                             Mask const& mask,
                             std::set<PickResult>& hits) {

  // first of all, check bbox
  auto box_hits(::gua::intersect(ray, bounding_box_));

  // ray did not intersect bbox -- therefore it wont intersect
  if (box_hits.first == Ray::END && box_hits.second == Ray::END) {
    return;
  }

  // return if only first object shall be returned and the current first hit
  // is in front of the bbox entry point and the ray does not start inside
  // the bbox
  if (options & PickResult::PICK_ONLY_FIRST_OBJECT && hits.size() > 0 &&
      hits.begin()->distance < box_hits.first && box_hits.first != Ray::END) {

    return;
  }

  auto geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

  if (geometry) {
    Ray world_ray(ray);
    geometry->ray_test(world_ray, options, this, hits);

  }
  
  for (auto child : get_children()) {
    child->ray_test_impl(ray, options, mask, hits);
  }

}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::update_bounding_box() const {
  if (geometry_) {
    auto geometry_bbox(geometry_->get_bounding_box());
    bounding_box_ = transform(geometry_bbox, world_transform_);

    for (auto child: get_children()) {
      bounding_box_.expandBy(child->get_bounding_box());
    }
  }
  else {
    Node::update_bounding_box();
  }
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::update_cache() {

  if(geometry_changed_) {
    if(geometry_description_ != "") {
      if(!GeometryDatabase::instance()->contains(geometry_description_)) {
        GeometryDescription desc(geometry_description_);
	try {
	  gua::PLODLoader loader;
	  loader.load_geometry(desc.filepath(), desc.flags());
	}
	catch (std::exception& e) {
	  Logger::LOG_WARNING << "PLODNode::update_cache(): Loading failed from " << desc.filepath() << " : " << e.what() << std::endl;
	}
      }
      geometry_ = std::dynamic_pointer_cast<PLODResource>(GeometryDatabase::instance()->lookup(geometry_description_));

      if(!geometry_) {
        Logger::LOG_WARNING << "Failed to get PLODResource for " << geometry_description_ << ": The data base entry is of wrong type!" << std::endl;
      }
    }

    geometry_changed_ = false;
  }

  // modified version of Node::upodate_cache -> add local transformation
  if (self_dirty_)
  {
    math::mat4 old_world_trans(world_transform_);

    if (is_root()) {
      world_transform_ = get_transform();
    }
    else {
      world_transform_ = get_parent()->get_world_transform() * transform_ * geometry_->local_transform();
    }

    if (world_transform_ != old_world_trans) {
      on_world_transform_changed.emit(world_transform_);
    }

    self_dirty_ = false;
  }

  if (child_dirty_) {
    for (auto const& child : get_children()) {
      child->update_cache();
    }

    update_bounding_box();

    child_dirty_ = false;
  }

}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void PLODNode::accept(NodeVisitor& visitor) {
  visitor.visit(this);
}

////////////////////////////////////////////////////////////////////////////////
std::shared_ptr<Node> PLODNode::copy() const {
  std::shared_ptr<PLODNode> result(new PLODNode(get_name(), geometry_description_, geometry_file_path_, material_, get_transform()));

  result->update_cache();

  result->shadow_mode_ = shadow_mode_;
  result->radius_scale_ = radius_scale_;
  result->error_threshold_ = error_threshold_;
  result->enable_backface_culling_by_normal_ = enable_backface_culling_by_normal_;

  return result;
}

}
}
