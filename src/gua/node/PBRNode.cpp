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
#include <gua/node/PBRNode.hpp>

#include <gua/databases/GeometryDatabase.hpp>
#include <gua/databases/MaterialDatabase.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/renderer/GeometryLoader.hpp>

// guacamole headers

namespace gua {

  ////////////////////////////////////////////////////////////////////////////////
  PBRNode::PBRNode(std::string const& name,
                           std::string const& filename,
                           std::string const& material,
                           math::mat4 const& transform)
    : GeometryNode(name, filename, material, transform)
  {}

  ////////////////////////////////////////////////////////////////////////////////

  void PBRNode::ray_test_impl(RayNode const& ray, PickResult::Options options,
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

    for (auto child : get_children()) {
      // test for intersection with each child
      child->ray_test_impl(ray, options, mask, hits);
    }

  }

  ////////////////////////////////////////////////////////////////////////////////
  std::shared_ptr<Node> PBRNode::copy() const {
    auto result(std::make_shared<PBRNode>(get_name(), filename_, material_, get_transform()));
    result->shadow_mode_ = shadow_mode_;
    return result;
  }
}
