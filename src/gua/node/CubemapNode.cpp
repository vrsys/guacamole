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
#include <gua/renderer/Frustum.hpp>

#include <limits>
#include <climits>

namespace gua {
namespace node {

CubemapNode::CubemapNode(std::string const& name,
                         Configuration const& configuration,
                         math::mat4 const& transform)
  : SerializableNode(name, transform), config(configuration) {}

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

  auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(texture_name_));
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


float CubemapNode::get_distance_by_local_direction(math::vec3 const& dir) const{

  math::mat4 screen_transform(scm::math::make_translation(0., 0., -0.5));
  std::vector<math::mat4> screen_transforms({
    screen_transform,
    scm::math::make_rotation(180., 0., 1., 0.) * screen_transform,
    scm::math::make_rotation(90., 1., 0., 0.) * screen_transform,
    scm::math::make_rotation(-90., 1., 0., 0.) * screen_transform,
    scm::math::make_rotation(90., 0., 1., 0.) * screen_transform,
    scm::math::make_rotation(-90., 0., 1., 0.) * screen_transform
  });

  std::vector<gua::Frustum> frusta;

  for(int i = 0; i<6; ++i){
    auto frustum(
      Frustum::perspective(
        math::mat4::identity(), screen_transforms[i],
        config.near_clip(),
        config.far_clip()
      )
    );
    frusta.push_back(frustum);
    if (frustum.contains(dir)){
      math::vec4 view_point(frustum.get_view() * math::vec4(dir.x, dir.y, dir.z, 1.0));
      math::vec4 proj_point(frustum.get_projection() * view_point);

      return acces_texture_data(i, math::vec2(proj_point.x, proj_point.y));
    }
  }

  return -1.0;
}

float CubemapNode::acces_texture_data(unsigned side, math::vec2 coords) const{
  unsigned row(config.resolution() * ((coords.x + 1)/2.0));
  unsigned colm(config.resolution() * ((coords.y + 1)/2.0));

  auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(texture_name_));
  if (texture){
    std::vector<float> const& v = texture->get_data();
    return v[row * config.resolution() * 6 + side * config.resolution() + colm];
  } else {
    return -1.0;
  }
}

std::shared_ptr<Node> CubemapNode::copy() const {
  return std::make_shared<CubemapNode>(*this);
}

}
}
