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
#include <gua/scenegraph/TexturedQuadNode.hpp>

// guacamole header
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {

TexturedQuadNode::TexturedQuadNode(std::string const& name,
                       Configuration const& configuration,
                       math::mat4 const& transform)
    : Node(name, transform), data(configuration) {}

/* virtual */ void TexturedQuadNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}

void TexturedQuadNode::update_bounding_box() const {
  math::BoundingBox<math::vec3> geometry_bbox(math::vec3(-0.5*data.get_size().x, -0.5*data.get_size().y, 0), math::vec3(0.5*data.get_size().x, 0.5*data.get_size().y, 0));
  bounding_box_ = transform(geometry_bbox, world_transform_);

  for (auto child : get_children()) {
      bounding_box_.expandBy(child->get_bounding_box());
  }

}

math::mat4 TexturedQuadNode::get_scaled_transform() const {
    math::mat4 scale(scm::math::make_scale(data.get_size().x, data.get_size().y, 1.f));
    return get_transform() * scale;
}

math::mat4 TexturedQuadNode::get_scaled_world_transform() const {
    math::mat4 scale(scm::math::make_scale(data.get_size().x, data.get_size().y, 1.f));
    return get_world_transform() * scale;
}

std::shared_ptr<Node> TexturedQuadNode::copy() const {
  return std::make_shared<TexturedQuadNode>(get_name(), data, get_transform());
}

}
