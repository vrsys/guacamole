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
#include <gua/node/TexturedScreenSpaceQuadNode.hpp>

// guacamole header
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/renderer/SerializedScene.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {
namespace node {

////////////////////////////////////////////////////////////////////////////////

TexturedScreenSpaceQuadNode::TexturedScreenSpaceQuadNode() {}

////////////////////////////////////////////////////////////////////////////////

TexturedScreenSpaceQuadNode::TexturedScreenSpaceQuadNode(
    std::string const& name,
    Configuration const& configuration)
    : SerializableNode(name),
      data(configuration) {}

/* virtual */ void TexturedScreenSpaceQuadNode::accept(NodeVisitor& visitor) {
  visitor.visit(this);
}

////////////////////////////////////////////////////////////////////////////////

void TexturedScreenSpaceQuadNode::update_bounding_box() const {
  math::BoundingBox<math::vec3> geometry_bbox(
      math::vec3(-0.5 * data.size().x, -0.5 * data.size().y, 0),
      math::vec3(0.5 * data.size().x, 0.5 * data.size().y, 0));

  bounding_box_ = transform(geometry_bbox, world_transform_);

  for (auto child : get_children()) {
    bounding_box_.expandBy(child->get_bounding_box());
  }
}

////////////////////////////////////////////////////////////////////////////////

void TexturedScreenSpaceQuadNode::update_cache() {
  Node::update_cache();

  if (!TextureDatabase::instance()->is_supported(data.texture())) {
    TextureDatabase::instance()->load(data.texture());
  }
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> TexturedScreenSpaceQuadNode::copy() const {
  return std::make_shared<TexturedScreenSpaceQuadNode>(get_name(), data);
}

////////////////////////////////////////////////////////////////////////////////

}
}
