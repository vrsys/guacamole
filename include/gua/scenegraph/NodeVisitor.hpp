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

namespace gua
{
namespace node
{
class Node;
class TransformNode;
class LODNode;
class SkeletalAnimationNode;
class TriMeshNode;
class TV_3Node;
class Video3DNode;
class VolumeNode;
class LightNode;
class ScreenNode;
class RayNode;
class SerializableNode;
class TexturedQuadNode;
class GeometryNode;
class ClippingPlaneNode;
} // namespace node

namespace physics
{
class RigidBodyNode;
class CollisionShapeNode;

} // namespace physics

/**
 * This class is used to recursively visit nodes in a SceneGraph
 *
 * \ingroup gua_scenegraph
 */
class NodeVisitor
{
  public:
    /**
     * Constructor.
     *
     * This constructs a NodeVisitor.
     */
    NodeVisitor() {}

    /**
     * Destructor.
     *
     * This destructs the NodeVisitor with all its contents.
     */
    virtual ~NodeVisitor() {}

    /**
     * Visits a Node.
     *
     * This function provides the interface to visit a Node.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a Node.
     */
    virtual void visit(node::Node* node){};

    /**
     * Visits a TransformNode.
     *
     * This function provides the interface to visit a TransformNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a TransformNode.
     */
    virtual void visit(node::TransformNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    /**
     * Visits an LODNode.
     *
     * This function provides the interface to visit an LODNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a LODNode.
     */
    virtual void visit(node::LODNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    /**
     * Visits an SkeletalAnimationNode.
     *
     * This function provides the interface to visit an SkeletalAnimationNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a SkeletalAnimationNode.
     */
    virtual void visit(node::SkeletalAnimationNode* node) { visit(reinterpret_cast<node::SerializableNode*>(node)); }

    /**
     * Visits a GeometryNode.
     *
     * This function provides the interface to visit a GeometryNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a GeometryNode.
     */
    virtual void visit(node::TriMeshNode* node) { visit(reinterpret_cast<node::SerializableNode*>(node)); }

    /**
     * Visits a GeometryNode.
     *
     * This function provides the interface to visit a GeometryNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a GeometryNode.
     */
    virtual void visit(node::GeometryNode* node) { visit(reinterpret_cast<node::SerializableNode*>(node)); }

    /**
     * Visits a Video3DNode
     *
     * This function provides the interface to visit a Video3DNode
     *
     * \param video3d   Pointer to Video3DNode
     */
    // virtual  void visit(node::Video3DNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    /**
     * Visits a GeometryNode
     * Visits a GeometryNode.
     *
     * This function provides the interface to visit a GeometryNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a GeometryNode.
     */
    // virtual void visit(node::VolumeNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    /**
     * Visits a LightNode.
     *
     * This function provides the interface to visit a LightNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a LightNode.
     */
    virtual void visit(node::LightNode* node) { visit(reinterpret_cast<node::SerializableNode*>(node)); }

    /**
     * Visits a ScreenNode.
     *
     * This function provides the interface to visit a ScreenNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a ScreenNode.
     */
    virtual void visit(node::ScreenNode* node) { visit(reinterpret_cast<node::SerializableNode*>(node)); }

    /**
     * Visits a RayNode.
     *
     * This function provides the interface to visit a ScreenNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a ScreenNode.
     */
    // virtual void visit(node::RayNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    virtual void visit(node::SerializableNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    /**
     * Visits a RayNode.
     *
     * This function provides the interface to visit a ScreenNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a ScreenNode.
     */
    // virtual void visit(node::RayNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    virtual void visit(node::RayNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    // #ifdef GUACAMOLE_ENABLE_PHYSICS
    /**
     * Visits a RigidBodyNode.
     *
     * This function provides the interface to visit a RigidBodyNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a RigidBodyNode.
     */
    virtual void visit(physics::RigidBodyNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

    /**
     * Visits a CollisionShapeNode.
     *
     * This function provides the interface to visit a CollisionShapeNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a CollisionShapeNode.
     */
    virtual void visit(physics::CollisionShapeNode* node) { visit(reinterpret_cast<node::Node*>(node)); }
    // #endif

    /**
     * Visits a TexturedQuadNode.
     *
     * This function provides the interface to visit a TexturedQuadNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a TexturedQuadNode.
     */
    virtual void visit(node::TexturedQuadNode* node) { visit(reinterpret_cast<node::SerializableNode*>(node)); }

    /**
     * Visits a ClippingPlaneNode.
     *
     * This function provides the interface to visit a ClippingPlaneNode.
     * Unless overwritten by derived classes, this defaults to visit(Node*).
     *
     * \param cam   Pointer to a ClippingPlaneNode.
     */
    virtual void visit(node::ClippingPlaneNode* node) { visit(reinterpret_cast<node::Node*>(node)); }

  private:
};

} // namespace gua

#endif // GUA_NODE_VISITOR_HPP
