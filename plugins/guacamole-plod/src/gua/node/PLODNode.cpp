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

// guacamole headers

namespace gua {
namespace node {

////////////////////////////////////////////////////////////////////////////////
PLODNode::PLODNode(std::string const& name,
                   std::string const& geometry_description,
                   std::string const& geometry_file_path,
                   Material const& material,
                   math::mat4 const& transform)
    : GeometryNode(name, transform),
      geometry_(nullptr),
      geometry_changed_(true),
      geometry_description_(geometry_description),
      geometry_file_path_(geometry_file_path),
      material_(material)
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
Material const& PLODNode::get_material() const {
  return material_;
}

////////////////////////////////////////////////////////////////////////////////
Material& PLODNode::get_material() {
  return material_;
}

////////////////////////////////////////////////////////////////////////////////
void PLODNode::set_material(Material const& material) {
  material_ = material;
  material_changed_ = self_dirty_ = true;
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

  // bbox is intersected, but check geometry only if mask tells us to check
  if (get_geometry_description() != "" && mask.check(get_tags())) {

    auto geometry(GeometryDatabase::instance()->lookup(get_geometry_description()));

    if (geometry) {

      bool check_kd_tree(true);

      math::mat4 world_transform(get_world_transform());

      // check for bounding box intersection of contained geometry if node
      // has children (in this case, the bbox might be larger
      // than the actual geometry)
      if (has_children()) {
        auto geometry_bbox(geometry->get_bounding_box());

#if 0
        auto inner_bbox = gua::math::transform(geometry_bbox, world_transform);
#else
        math::BoundingBox<math::vec3> inner_bbox;
        inner_bbox.expandBy(world_transform * geometry_bbox.min);
        inner_bbox.expandBy(world_transform * geometry_bbox.max);
        inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.min.x,
                                                         geometry_bbox.min.y,
                                                         geometry_bbox.max.z));
        inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.min.x,
                                                         geometry_bbox.max.y,
                                                         geometry_bbox.min.z));
        inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.min.x,
                                                         geometry_bbox.max.y,
                                                         geometry_bbox.max.z));
        inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.max.x,
                                                         geometry_bbox.min.y,
                                                         geometry_bbox.max.z));
        inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.max.x,
                                                         geometry_bbox.max.y,
                                                         geometry_bbox.min.z));
        inner_bbox.expandBy(world_transform * math::vec3(geometry_bbox.max.x,
                                                         geometry_bbox.min.y,
                                                         geometry_bbox.min.z));
#endif

        auto inner_hits(::gua::intersect(ray, inner_bbox));
        if (inner_hits.first == Ray::END &&
            inner_hits.second == Ray::END)
          check_kd_tree = false;
      }

      if (check_kd_tree) {
        Ray world_ray(ray);

        math::mat4 ori_transform(scm::math::inverse(world_transform));

        math::vec4 ori(world_ray.origin_[0],
                       world_ray.origin_[1],
                       world_ray.origin_[2],
                       1.0);
        math::vec4 dir(world_ray.direction_[0],
                       world_ray.direction_[1],
                       world_ray.direction_[2],
                       0.0);

        ori = ori_transform * ori;
        dir = ori_transform * dir;

        Ray object_ray(ori, dir, world_ray.t_max_);
        geometry->ray_test(object_ray, options, this, hits);

        float const inf(std::numeric_limits<float>::max());

        if (options & PickResult::GET_WORLD_POSITIONS) {

          for (auto& hit : hits) {
            if (hit.world_position == math::vec3(inf, inf, inf)) {
              auto transformed(
                  world_transform *
                  math::vec4(
                      hit.position.x, hit.position.y, hit.position.z, 0.0));
              hit.world_position =
                  scm::math::vec3(transformed.x, transformed.y, transformed.z);
            }
          }
        }

        if (options & PickResult::GET_WORLD_NORMALS) {

          math::mat4 normal_matrix(
              scm::math::inverse(scm::math::transpose(world_transform)));
          for (auto& hit : hits) {
            if (hit.world_normal == math::vec3(inf, inf, inf)) {
              auto transformed(
                  normal_matrix *
                  math::vec4(hit.normal.x, hit.normal.y, hit.normal.z, 0.0));
              hit.world_normal = scm::math::normalize(
                  scm::math::vec3(transformed.x, transformed.y, transformed.z));
            }
          }
        }
      }
    }
  }

  for (auto child : get_children()) {
    // test for intersection with each child
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

  // The code below auto-loads a material if it's not already supported by
  // the MaterialShaderDatabase. It expects a material name like
  // 
  // data/materials/ShadelessPLOD.gmd

  if(material_changed_) {
    if (material_.get_shader_name() != "") {
      //to fill
    }

    material_changed_ = false;
  }

  GeometryNode::update_cache();
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

  return result;
}

}
}
