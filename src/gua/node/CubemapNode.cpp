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
#include <gua/node/CubemapNode.hpp>

// guacamole headers
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/databases.hpp>
#include <gua/renderer/TextureDistance.hpp>

#include <limits>
#include <climits>

namespace gua {
namespace node {

CubemapNode::CubemapNode(std::string const& name, math::mat4 const& transform)
    : SerializableNode(name, transform) {
      texture_name_ = name + "_texture";
    }

/* virtual */ void CubemapNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}

void CubemapNode::set_texture_name(std::string const& name){
  // TODO remove old Texture in database
  texture_name_ = name;
}

std::string CubemapNode::get_texture_name() const{
  // TODO create new texture in database
  return texture_name_;
}

float CubemapNode::get_closest_distance() const{

  auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup("DepthCubeMapTestTexture"));
  if (texture){
    std::vector<float> const& v = texture->get_data();

    float closest(std::numeric_limits<float>::max());
    for (const float &f : v){
      if ( (f < closest) && (f!=-1.f) ){
        closest = f;
      }
    }
    return closest;

  }
  return -1.0f;

}

std::shared_ptr<Node> CubemapNode::copy() const {
  return std::make_shared<CubemapNode>(*this);
}

}
}
