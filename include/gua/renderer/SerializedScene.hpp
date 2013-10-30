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
#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/scenegraph/PointLightNode.hpp>
#include <gua/scenegraph/SpotLightNode.hpp>
#include <gua/scenegraph/ScreenNode.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/scenegraph/TexturedQuadNode.hpp>
#include <gua/renderer/SerializedNode.hpp>
#include <gua/math/BoundingBox.hpp>
#include <gua/renderer/Frustum.hpp>

// external headers
#include <vector>
#include <string>
#include <map>
#include <set>

namespace gua {

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
  std::vector<SerializedNode<GeometryNode::Configuration> > meshnodes_;

  /**
   * All NURBS nodes.
   */
  std::vector<SerializedNode<GeometryNode::Configuration> > nurbsnodes_;

  /**
   * All point light nodes.
   */
  std::vector<SerializedNode<PointLightNode::Configuration> > point_lights_;

  /**
   * All spot light nodes.
   */
  std::vector<SerializedNode<SpotLightNode::Configuration> > spot_lights_;

  /**
   * The frustum.
   */
  Frustum frustum;

  /**
   * All used materials.
   */
  std::set<std::string> materials_;

  /**
   * All bounding boxes.
   */
  std::vector<math::BoundingBox<math::vec3> > bounding_boxes_;

  /**
   * All bounding boxes.
   */
  std::vector<SerializedNode<GeometryNode::Configuration> > rays_;

  /**
   * All textured quads.
   */
  std::vector<SerializedNode<TexturedQuadNode::Configuration> > textured_quads_;
};

}

#endif  // GUA_SERIALIZED_SCENE_HPP
