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

#ifndef GUA_NODE_VISITOR_HPP
#define GUA_NODE_VISITOR_HPP

// guacamole headers

// external headers

namespace gua {

class Node;
class TransformNode;
class LODNode;
class GeometryNode;
class Video3DNode;
class VolumeNode;
class PointLightNode;
class ScreenNode;
class SpotLightNode;
class SunLightNode;
class RayNode;
class TexturedQuadNode;

namespace physics {

class RigidBodyNode;
class CollisionShapeNode;

}

/**
 * This class is used to recursively visit nodes in a scenegraph
 *
 */
class NodeVisitor {
 public:

  /**
   * Constructor.
   *
   * This constructs a NodeVisitor.
   */
  NodeVisitor();

  /**
   * Destructor.
   *
   * This destructs the NodeVisitor with all its contents.
   */
  virtual ~NodeVisitor();

  /**
   * Visits a TransformNode
   *
   * This function provides the interface to visit a TransformNode
   *
   * \param cam   Pointer to TransformNode
   */
  virtual void visit(Node* node) {};

  /**
   * Visits a TransformNode
   *
   * This function provides the interface to visit a TransformNode
   *
   * \param cam   Pointer to TransformNode
   */
  virtual void visit(TransformNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits an LODNode
   *
   * This function provides the interface to visit an LODNode
   *
   * \param cam   Pointer to LODNode
   */
  virtual void visit(LODNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a GeometryNode
   *
   * This function provides the interface to visit a GeometryNode
   *
   * \param cam   Pointer to GeometryNode
   */
  virtual void visit(GeometryNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
  * Visits a Video3DNode
  *
  * This function provides the interface to visit a Video3DNode
  *
  * \param video3d   Pointer to Video3DNode
  */
  virtual  void visit(Video3DNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
  * Visits a GeometryNode
  *
  * This function provides the interface to visit a GeometryNode
  *
  * \param cam   Pointer to GeometryNode
  */
  virtual void visit(VolumeNode* node) { visit(reinterpret_cast<Node*>(node)); }


  /**
   * Visits a PointLightNode
   *
   * This function provides the interface to visit a PointLightNode
   *
   * \param cam   Pointer to PointLightNode
   */
  virtual void visit(PointLightNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a SpotLightNode
   *
   * This function provides the interface to visit a SpotLightNode
   *
   * \param cam   Pointer to SpotLightNode
   */
  virtual void visit(SpotLightNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a SunLightNode
   *
   * This function provides the interface to visit a SunLightNode
   *
   * \param cam   Pointer to SunLightNode
   */
  virtual void visit(SunLightNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a SpotLightNode
   *
   * This function provides the interface to visit a ScreenNode
   *
   * \param cam   Pointer to ScreenNode
   */
  virtual void visit(ScreenNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a RayNode
   *
   * This function provides the interface to visit a ScreenNode
   *
   * \param cam   Pointer to ScreenNode
   */
  virtual void visit(RayNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a RigidBodyNode
   *
   * This function provides the interface to visit a RigidBodyNode
   *
   * \param cam   Pointer to RigidBodyNode
   */
  virtual void visit(physics::RigidBodyNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a CollisionShapeNode
   *
   * This function provides the interface to visit a CollisionShapeNode
   *
   * \param cam   Pointer to CollisionShapeNode
   */
  virtual void visit(physics::CollisionShapeNode* node) { visit(reinterpret_cast<Node*>(node)); }

  /**
   * Visits a TexturedQuadNode
   *
   * This function provides the interface to visit a TexturedQuadNode
   *
   * \param cam   Pointer to TexturedQuadNode
   */
  virtual void visit(TexturedQuadNode* node) { visit(reinterpret_cast<Node*>(node)); }

 private:

};

}

#endif  // GUA_NODE_VISITOR_HPP
