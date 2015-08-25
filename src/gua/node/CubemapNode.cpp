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
  : SerializableNode(name, transform), config(configuration)
{
  m_NewTextureData = std::make_shared<std::atomic<bool>>(false);
  m_MinDistance.distance = -1.0;
}

/* virtual */ void CubemapNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}

float CubemapNode::get_min_distance(){
  if (m_NewTextureData->load()){
    find_min_distance();
  }
  return m_MinDistance.distance;
}

math::vec3 CubemapNode::get_min_distance_position(){
  if (m_NewTextureData->load()){
    find_min_distance();
  }
  return m_MinDistance.world_position;
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

  for(int i = 0; i<6; ++i){
    auto frustum(
      Frustum::perspective(
        math::mat4::identity(), screen_transforms[i],
        config.near_clip(),
        config.far_clip()
      )
    );
    if (frustum.contains(dir)){
      math::vec4 view_point(frustum.get_view() * math::vec4(dir.x, dir.y, dir.z, 1.0));
      math::vec4 proj_point(frustum.get_projection() * view_point);

      return acces_texture_data(i, math::vec2(proj_point.x, proj_point.y));
    }
  }

  return -1.0;
}

void CubemapNode::find_min_distance(){
  auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(config.get_texture_name()));
  if (texture){
    std::vector<float> const& data = texture->get_data();
    *(m_NewTextureData) = false;

    Distance_Info min_distance;
    min_distance.distance = std::numeric_limits<float>::max();

    for (const float &f : data){
      if ( (f < min_distance.distance) && (f!=-1.f) ){
        min_distance.distance = f;
        uint index = &f - &data[0];
        min_distance.tex_coords = math::vec2(index%(config.resolution()*6), index/(config.resolution()*6));
      }
    }

    if (min_distance.distance != std::numeric_limits<float>::max()){
      min_distance.world_position = project_back_to_world_coords(min_distance);
      m_MinDistance = min_distance;
      return;
    }

  }
  m_MinDistance.distance = -1.0;
}

math::vec3 CubemapNode::project_back_to_world_coords(Distance_Info const& di) const{
  int side = di.tex_coords.x / config.resolution();

  math::vec2 xy( float(di.tex_coords.x) / config.resolution(), float(di.tex_coords.y) / config.resolution() ); 
  xy -= 0.5;

  math::vec4 point_on_face( world_transform_ *  math::vec4( xy.x, xy.y, -0.5, 1.0) );
  math::vec3 center( gua::math::get_translation(world_transform_) );
  math::vec3 direction( math::vec3(point_on_face.x, point_on_face.y, point_on_face.z) - center);
  direction = scm::math::normalize(direction);

  return center + (direction*di.distance);
}

float CubemapNode::acces_texture_data(unsigned side, math::vec2 coords) const{
  unsigned row(config.resolution() * ((coords.y + 1)/2.0));
  unsigned colm(config.resolution() * ((coords.x + 1)/2.0));

  auto texture = std::dynamic_pointer_cast<TextureDistance>(TextureDatabase::instance()->lookup(config.get_texture_name()));
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
