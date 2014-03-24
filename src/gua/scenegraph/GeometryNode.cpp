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
#include <gua/scenegraph/GeometryNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/renderer/GeometryLoader.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {

GeometryNode::GeometryNode(std::string const& name,
                           std::string const& geometry,
                           std::string const& material,
                           math::mat4 const& transform)
    : Node(name, transform), geometry_(geometry), material_(material),
      geometry_changed_(false), material_changed_(false) {}

/* virtual */ void GeometryNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}

////////////////////////////////////////////////////////////////////////////////

void GeometryNode::update_bounding_box() const {

  if (get_geometry() != "") {
    auto geometry_bbox(GeometryDatabase::instance()->lookup(get_geometry())->get_bounding_box());
    bounding_box_ = transform(geometry_bbox, world_transform_);

    for (auto child : get_children()) {
      bounding_box_.expandBy(child->get_bounding_box());
    }
  }
  else {
    Node::update_bounding_box();
  }
}

////////////////////////////////////////////////////////////////////////////////

void GeometryNode::update_cache() {

    // The code below auto-loads a geometry if it's not already supported by
    // the GeometryDatabase. It expects a geometry name like
    //
    // "type='file'&file='data/objects/monkey.obj'&id=0&flags=0"

    if (geometry_changed_) {
      if (geometry_ != "") {
        if (!GeometryDatabase::instance()->is_supported(geometry_)) {
          auto params(string_utils::split(geometry_, '&'));
          if (params.size() == 4) {
            if (params[0] == "type=file") {
              auto tmp_filename(string_utils::split(params[1], '='));
              auto tmp_flags(string_utils::split(params[3], '='));
              if (tmp_filename.size() == 2 && tmp_flags.size() == 2) {
                std::string filename(tmp_filename[1]);
                std::string flags_string(tmp_flags[1]);
                unsigned flags(0);
                std::stringstream sstr(flags_string);
                sstr >> flags;

                GeometryLoader loader;
                loader.load_geometry(filename, flags);

              } else {
                Logger::LOG_WARNING << "Failed to auto-load geometry " << geometry_ << ": Failed to extract filename and/or loading flags!" << std::endl;
              }
            } else {
              Logger::LOG_WARNING << "Failed to auto-load geometry " << geometry_ << ": Type is not supported!" << std::endl;
            }
          } else {
            Logger::LOG_WARNING << "Failed to auto-load geometry " << geometry_ << ": The name does not contain a type, file, id and flag parameter!" << std::endl;
          }
        }
      }

      geometry_changed_ = false;
    }

    // The code below auto-loads a material if it's not already supported by
    // the MaterialDatabase. It expects a material name like
    //
    // data/materials/Stones.gmd

    if (material_changed_) {
      if (material_ != "") {
        if (!MaterialDatabase::instance()->is_supported(material_)) {
          auto mat = std::make_shared<Material>(material_, MaterialDescription(material_));
          MaterialDatabase::instance()->add(material_, mat);
        }
      }

      material_changed_ = false;
    }

    Node::update_cache();
}

////////////////////////////////////////////////////////////////////////////////

void GeometryNode::ray_test_impl(RayNode const& ray, PickResult::Options options,
                           Mask const& mask, std::set<PickResult>& hits) {

  // first of all, check bbox
  auto box_hits(ray.intersect(bounding_box_));

  // ray did not intersect bbox -- therefore it wont intersect
  if (box_hits.first == RayNode::END && box_hits.second == RayNode::END) {
    return;
  }

  // return if only first object shall be returned and the current first hit
  // is in front of the bbox entry point and the ray does not start inside
  // the bbox
  if (options & PickResult::PICK_ONLY_FIRST_OBJECT
      && hits.size() > 0 && hits.begin()->distance < box_hits.first
      && box_hits.first != Ray::END) {

    return;
  }

  // bbox is intersected, but check geometry only if mask tells us to check
  if (get_geometry() != "" && mask.check(get_groups())) {

    auto geometry(GeometryDatabase::instance()->lookup(get_geometry()));

    if (geometry) {

      bool check_kd_tree(true);

      math::mat4 world_transform(get_world_transform());

      // check for bounding box intersection of contained geometry if node
      // has children (in this case, the bbox might be larger
      // than the actual geometry)
      if (has_children()) {
        auto geometry_bbox(geometry->get_bounding_box());

        math::BoundingBox<math::vec3> inner_bbox;
        inner_bbox.expandBy(world_transform * geometry_bbox.min);
        inner_bbox.expandBy(world_transform * geometry_bbox.max);
        inner_bbox.expandBy(world_transform *
                            math::vec3(geometry_bbox.min.x,
                                       geometry_bbox.min.y,
                                       geometry_bbox.max.z));
        inner_bbox.expandBy(world_transform *
                            math::vec3(geometry_bbox.min.x,
                                       geometry_bbox.max.y,
                                       geometry_bbox.min.z));
        inner_bbox.expandBy(world_transform *
                            math::vec3(geometry_bbox.min.x,
                                       geometry_bbox.max.y,
                                       geometry_bbox.max.z));
        inner_bbox.expandBy(world_transform *
                            math::vec3(geometry_bbox.max.x,
                                       geometry_bbox.min.y,
                                       geometry_bbox.max.z));
        inner_bbox.expandBy(world_transform *
                            math::vec3(geometry_bbox.max.x,
                                       geometry_bbox.max.y,
                                       geometry_bbox.min.z));
        inner_bbox.expandBy(world_transform *
                            math::vec3(geometry_bbox.max.x,
                                       geometry_bbox.min.y,
                                       geometry_bbox.min.z));

        auto inner_hits(ray.intersect(inner_bbox));
        if (inner_hits.first == RayNode::END &&
            inner_hits.second == RayNode::END)
          check_kd_tree = false;
      }

      if (check_kd_tree) {
        Ray world_ray(ray.get_world_ray());

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

          for (auto& hit: hits) {
            if (hit.world_position == math::vec3(inf, inf, inf)) {
              auto transformed(world_transform * math::vec4(hit.position.x, hit.position.y, hit.position.z, 0.0));
              hit.world_position = scm::math::vec3(transformed.x, transformed.y, transformed.z);
            }
          }
        }

        if (options & PickResult::GET_WORLD_NORMALS) {

          math::mat4 normal_matrix(scm::math::inverse(scm::math::transpose(world_transform)));
          for (auto& hit: hits) {
            if (hit.world_normal == math::vec3(inf, inf, inf)) {
              auto transformed(normal_matrix * math::vec4(hit.normal.x, hit.normal.y, hit.normal.z, 0.0));
              hit.world_normal = scm::math::normalize(scm::math::vec3(transformed.x, transformed.y, transformed.z));
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

std::shared_ptr<Node> GeometryNode::copy() const {
  return std::make_shared<GeometryNode>(get_name(), geometry_, material_, get_transform());
}

}
