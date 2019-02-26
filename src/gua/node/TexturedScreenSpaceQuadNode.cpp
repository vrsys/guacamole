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

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////

TexturedScreenSpaceQuadNode::TexturedScreenSpaceQuadNode() {}

////////////////////////////////////////////////////////////////////////////////

TexturedScreenSpaceQuadNode::TexturedScreenSpaceQuadNode(std::string const& name, Configuration const& configuration) : SerializableNode(name), data(configuration) {}

/* virtual */ void TexturedScreenSpaceQuadNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////

bool TexturedScreenSpaceQuadNode::pixel_to_texcoords(math::vec2 const& pixel, math::vec2ui const& screen_size, math::vec2& result) const
{
    math::vec2 pos(pixel / screen_size * 2 - 1);

    math::vec2 size(1.0 * data.get_size().x / screen_size.x, 1.0 * data.get_size().y / screen_size.y);

    math::vec2 offset((2.0 * data.get_offset().x + data.get_anchor().x * (screen_size.x - data.get_size().x)) / screen_size.x,
                      (2.0 * data.get_offset().y + data.get_anchor().y * (screen_size.y - data.get_size().y)) / screen_size.y);

    pos -= offset;
    pos /= size;

    result = pos * 0.5 + 0.5;

    return result.x >= 0 && result.x < 1 && result.y >= 0 && result.y < 1;
}

////////////////////////////////////////////////////////////////////////////////

void TexturedScreenSpaceQuadNode::update_bounding_box() const { bounding_box_ = math::BoundingBox<math::vec3>(); }

////////////////////////////////////////////////////////////////////////////////

void TexturedScreenSpaceQuadNode::update_cache()
{
    Node::update_cache();

    if(!TextureDatabase::instance()->contains(data.texture()))
    {
        TextureDatabase::instance()->load(data.texture());
    }
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> TexturedScreenSpaceQuadNode::copy() const { return std::make_shared<TexturedScreenSpaceQuadNode>(*this); }

////////////////////////////////////////////////////////////////////////////////

} // namespace node
} // namespace gua
