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
#include <gua/node/Node.hpp>
#include <gua/node/ScreenNode.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/renderer/Frustum.hpp>

// external headers
#include <vector>
#include <unordered_map>
#include <typeindex>

namespace gua {

/**
 * Stores a serialized scene graph.
 *
 * When the optimizer traverses the scene graph, it produces an SerializedScene
 * which contains relevant nodes only.
 */
struct GUA_DLL SerializedScene {

  /**
  * All geometry nodes.
  */
  std::unordered_map<std::type_index, std::vector<node::Node*>> nodes;

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
  std::vector<math::BoundingBox<math::vec3> > bounding_boxes;
};

}

#endif  // GUA_SERIALIZED_SCENE_HPP
