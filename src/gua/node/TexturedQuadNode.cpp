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
#include <gua/node/TexturedQuadNode.hpp>

// guacamole header
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/utils/Logger.hpp>
#include <gua/databases/TextureDatabase.hpp>
#include <gua/math/BoundingBoxAlgo.hpp>

namespace gua {
namespace node {

TexturedQuadNode::TexturedQuadNode()
    : texture_(""),
      size_(1.0f, 1.0f),
      is_stereo_texture_(false),
      flip_x_(false),
      flip_y_(false) {}

TexturedQuadNode::TexturedQuadNode(std::string const& name,
                                   std::string const& texture,
                                   math::mat4 const& transform,
                                   math::vec2 const& size,
                                   bool is_stereo,
                                   bool flipx,
                                   bool flipy)
    : Node(name, transform),
      texture_(texture),
      size_(size),
      is_stereo_texture_(is_stereo),
      flip_x_(flipx),
      flip_y_(flipy) {}

/* virtual */ void TexturedQuadNode::accept(NodeVisitor& visitor) {
  visitor.visit(this);
}

void TexturedQuadNode::update_bounding_box() const {
  math::BoundingBox<math::vec3> geometry_bbox(
      math::vec3(-0.5 * get_size().x, -0.5 * get_size().y, 0),
      math::vec3(0.5 * get_size().x, 0.5 * get_size().y, 0));

  bounding_box_ = transform(geometry_bbox, world_transform_);

  for (auto child : get_children()) {
    bounding_box_.expandBy(child->get_bounding_box());
  }
}

void TexturedQuadNode::update_cache() {
  Node::update_cache();

  if (!TextureDatabase::instance()->is_supported(texture_)) {
    TextureDatabase::instance()->load(texture_);
  }
}

math::mat4 TexturedQuadNode::get_scaled_transform() const {
  math::mat4 scale(scm::math::make_scale(get_size().x, get_size().y, 1.f));
  return get_transform() * scale;
}

math::mat4 TexturedQuadNode::get_scaled_world_transform() const {
  math::mat4 scale(scm::math::make_scale(get_size().x, get_size().y, 1.f));
  return get_world_transform() * scale;
}

std::shared_ptr<Node> TexturedQuadNode::copy() const {
  return std::make_shared<TexturedQuadNode>(get_name(),
                                            texture_,
                                            get_transform(),
                                            size_,
                                            is_stereo_texture_,
                                            flip_x_,
                                            flip_y_);
}

std::string const& TexturedQuadNode::get_texture() const { return texture_; }

void TexturedQuadNode::set_texture(std::string const& name) {
  texture_ = name;
  update_cache();
}

math::vec2 const& TexturedQuadNode::get_size() const { return size_; }

void TexturedQuadNode::get_size(math::vec2 const& size) { size_ = size; }

bool TexturedQuadNode::is_stereo_texture() const { return is_stereo_texture_; }

void TexturedQuadNode::is_stereo_texture(bool enable) {
  is_stereo_texture_ = enable;
}

bool TexturedQuadNode::flip_x() const { return flip_x_; }

void TexturedQuadNode::flip_x(bool enable) { flip_x_ = enable; }

bool TexturedQuadNode::flip_y() const { return flip_y_; }

void TexturedQuadNode::flip_y(bool enable) { flip_y_ = enable; }

}
}
