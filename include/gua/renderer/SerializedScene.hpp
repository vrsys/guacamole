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

#ifndef GUA_SERIALIZED_SCENE_HPP
#define GUA_SERIALIZED_SCENE_HPP

// guacamole headers
#include <gua/node/GeometryNode.hpp>
#include <gua/node/Video3DNode.hpp>
#include <gua/node/VolumeNode.hpp>
#include <gua/node/PointLightNode.hpp>
#include <gua/node/SpotLightNode.hpp>
#include <gua/node/SunLightNode.hpp>
#include <gua/node/ScreenNode.hpp>
#include <gua/node/RayNode.hpp>
#include <gua/node/TexturedQuadNode.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/renderer/Frustum.hpp>

// external headers
#include <vector>
#include <string>
#include <map>
#include <set>
#include <unordered_map>
#include <typeindex>

namespace gua {

class UberShader;

/**
 * Stores a serialized scene graph.
 *
 * When the optimizer traverses the scene graph, it produces an SerializedScene
 * which contains relevant nodes only.
 */
struct SerializedScene {

  /**
  * All geometry nodes.
  */
  std::unordered_map<std::type_index, std::unordered_multimap<std::string, node::GeometryNode*>> geometrynodes_;

  /**
  * All Volume nodes.
  */
  std::vector<node::VolumeNode*> volumenodes_;

  /**
   * All point light nodes.
   */
  std::vector<node::PointLightNode*> point_lights_;

  /**
   * All spot light nodes.
   */
  std::vector<node::SpotLightNode*> spot_lights_;

  /**
   * All sun light nodes.
   */
  std::vector<node::SunLightNode*> sun_lights_;

  /**
   * The frustum.
   */
  Frustum frustum;

  /**
   * The center of interest.
   */
  math::vec3 center_of_interest;

  /**
   * All bounding boxes.
   */
  std::vector<math::BoundingBox<math::vec3> > bounding_boxes_;

  /**
   * All bounding boxes.
   */
  std::vector<node::RayNode*> rays_;

  /**
   * All textured quads.
   */
  std::vector<node::TexturedQuadNode*> textured_quads_;
};

}

#endif  // GUA_SERIALIZED_SCENE_HPP
