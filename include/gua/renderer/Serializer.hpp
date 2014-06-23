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

#ifndef GUA_SERIALIZER_HPP
#define GUA_SERIALIZER_HPP

#include <stack>

// guacamole headers
#include <gua/renderer/SerializedScene.hpp>
#include <gua/renderer/Camera.hpp>
#include <gua/renderer/Frustum.hpp>
#include <gua/renderer/enums.hpp>
#include <gua/utils/Mask.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>

namespace gua {

class SceneGraph;

/**
 * This class is used to convert the scengraph to a (opimized) sequence.
 *
 * It serializes the scene graph.
 */
class Serializer : public NodeVisitor {
 public:

  /**
   * Constructor.
   *
   * This constructs an Serializer.
   */
  Serializer();

  /**
   * Takes the Scengraph and processes geometry, light and camera
   *        lists.
   *
   * \param scene_graph          The SceneGraph to be processed.
   * \param render_mask          The mask to be applied to the nodes of
   *                             the graph.
   */
  void check(SerializedScene& output,
             SceneGraph const& scene_graph,
             std::string const& render_mask,
             bool draw_bounding_boxes,
             bool draw_rays,
             bool enable_frustum_culling);

  /**
   * Visits a TransformNode
   *
   * This function provides the interface to visit a TransformNode
   *
   * \param cam   Pointer to TransformNode
   */
  /* virtual */ void visit(Node* node);

  /**
   * Visits an LODNode
   *
   * This function provides the interface to visit an LODNode
   *
   * \param cam   Pointer to LODNode
   */
  /* virtual */ void visit(LODNode* lod);

  /**
   * Visits a GeometryNode
   *
   * This function provides the interface to visit a GeometryNode
   *
   * \param geometry   Pointer to GeometryNode
   */
  /* virtual */ void visit(GeometryNode* geometry);

  /**
  * Visits a VolumeNode
  *
  * This function provides the interface to visit a VolumeNode
  *
  * \param volume   Pointer to VolumeNode
  */
  /* virtual */ void visit(VolumeNode* volume);

  /**
   * Visits a PointLightNode
   *
   * This function provides the interface to visit a PointLightNode
   *
   * \param pointlight   Pointer to PointLightNode
   */
  /* virtual */ void visit(PointLightNode* pointlight);

  /**
   * Visits a SpotLightNode
   *
   * This function provides the interface to visit a SpotLightNode
   *
   * \param spot   Pointer to SpotLightNode
   */
  /* virtual */ void visit(SpotLightNode* spot);

   /**
   * Visits a SunLightNode
   *
   * This function provides the interface to visit a SunLightNode
   *
   * \param spot   Pointer to SunLightNode
   */
  /* virtual */ void visit(SunLightNode* sun);

  /**
   * Visits a RayNode
   *
   * This function provides the interface to visit a RayNode
   *
   * \param spot   Pointer to RayNode
   */
  /* virtual */ void visit(RayNode* ray);

  /**
   * Visits a RigidBodyNode
   *
   * This function provides the interface to visit a RigidBodyNode
   *
   * \param cam   Pointer to RigidBodyNode
   */
  /* virtual */ void visit(physics::RigidBodyNode*) {}

  /**
   * Visits a CollisionShapeNode
   *
   * This function provides the interface to visit a CollisionShapeNode
   *
   * \param cam   Pointer to CollisionShapeNode
   */
  /* virtual */ void visit(physics::CollisionShapeNode*) {}

  /**
   * Visits a TexturedQuadNode
   *
   * This function provides the interface to visit a TexturedQuadNode
   *
   * \param node  Pointer to TexturedQuadNode
   */
  /* virtual */ void visit(TexturedQuadNode* node);

 private:

  bool is_visible(Node* node) const;
  void add_bbox(Node* node) const;
  void visit_children(Node* node);

  Frustum current_frustum_;
  Mask current_render_mask_;
  math::vec3 current_center_of_interest_;

  SerializedScene* data_;
  bool draw_bounding_boxes_;
  bool draw_rays_;
  bool enable_frustum_culling_;

};

}

#endif  // GUA_SERIALIZER_HPP
