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
#include <gua/node/VolumeNode.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/databases/GeometryDatabase.hpp>
#include <gua/renderer/VolumeLoader.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {

  /////////////////////////////////////////////////////////////////////////////

  VolumeNode::VolumeNode(std::string const& name,
                           Configuration const& configuration,
                           math::mat4 const& transform)
    : Node(name, transform), data(configuration) {}

  /////////////////////////////////////////////////////////////////////////////

  /* virtual */ void VolumeNode::accept(NodeVisitor& visitor) {
    visitor.visit(this);
  }

  /////////////////////////////////////////////////////////////////////////////

  void VolumeNode::update_bounding_box() const {

    if (data.get_volume() != "") {

      auto geometry_bbox(GeometryDatabase::instance()->lookup(data.get_volume())->get_bounding_box());
      bounding_box_ = transform(geometry_bbox, world_transform_);

      for (auto child : get_children()) {
        bounding_box_.expandBy(child->get_bounding_box());
      }
    } else {
      Node::update_bounding_box();
    }
  }

  /////////////////////////////////////////////////////////////////////////////

  void VolumeNode::ray_test_impl(RayNode const& ray, PickResult::Options options,
                           Mask const& mask, std::set<PickResult>& hits) {

    // first of all, check bbox
    auto box_hits(ray.intersect(bounding_box_));
  }

  /////////////////////////////////////////////////////////////////////////////

  std::shared_ptr<Node> VolumeNode::copy() const {
    return std::make_shared<VolumeNode>(get_name(), data, get_transform());
  }

} // namespace gua
