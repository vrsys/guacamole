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

#ifndef GUA_DOT_GENERATOR_HPP
#define GUA_DOT_GENERATOR_HPP

#include <map>
#include <vector>
#include <string>

// guacamole header
#include <gua/platform.hpp>
#include <gua/scenegraph/NodeVisitor.hpp>

namespace gua {

class SceneGraph;

namespace node {
class Node;
class GeometryNode;
class PointLightNode;
class ScreenNode;
class SpotLightNode;
}

/**
 * This class may be used to parse a path.
 */
class GUA_DLL DotGenerator : public NodeVisitor {
 public:

  DotGenerator();
  ~DotGenerator();

  /**
   * Parses a graph.
   *
   * This function parses a SceneGraph and generates a graph in
   * DOT-syntax. The graph then can be saved to a file with the
   * save() method.
   *
   * \param graph       The graph to be parsed.
   */
  void parse_graph(SceneGraph const* graph);

  ///@{
  /**
   * Visiters for each Node type
   */
  /*virtual*/ void visit(node::Node* node);
  /*virtual*/ void visit(node::TransformNode* cam);
  /*virtual*/ void visit(node::GeometryNode* geometry);
  /*virtual*/ void visit(node::VolumeNode* volume);
  /*virtual*/ void visit(node::PointLightNode* pointlight);
  /*virtual*/ void visit(node::ScreenNode* screen);
  /*virtual*/ void visit(node::SpotLightNode* spotlight);
  /*virtual*/ void visit(node::RayNode* ray);
#ifdef GUACAMOLE_ENABLE_PHYSICS
  /*virtual*/ void visit(physics::RigidBodyNode* rb);
  /*virtual*/ void visit(physics::CollisionShapeNode* shape);
#endif
  /*virtual*/ void visit(node::TexturedQuadNode* node);
   ///@}

  /**
   * Saves a DOT-file
   *
   * This function saves the generated DOT-graph.
   *
   * \param path_to_file  The name of the file the DOT-graph will be saved to.
   *                      The ending has to be .gv or .dot.
   */
  void save(std::string const& path_to_file) const;

 private:

  void pre_node_info(node::Node*);
  void post_node_info(node::Node*, std::string const& fillcolor);

  std::string parse_data_;
  std::string graph_name_;

  std::map<int, int> added_nodes_;
  std::size_t node_count_;

};

}

#endif  //DOT_GENERATOR_HPP
