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
#include <gua/utils/DotGenerator.hpp>

// guacamole headers
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/scenegraph/TransformNode.hpp>
#include <gua/scenegraph/GeometryNode.hpp>
#include <gua/scenegraph/VolumeNode.hpp>
#include <gua/scenegraph/PointLightNode.hpp>
#include <gua/scenegraph/SpotLightNode.hpp>
#include <gua/scenegraph/ScreenNode.hpp>
#include <gua/scenegraph/RayNode.hpp>
#include <gua/scenegraph/TexturedQuadNode.hpp>
#if GUACAMOLE_ENABLE_PHYSICS
#include <gua/physics/RigidBodyNode.hpp>
#include <gua/physics/CollisionShapeNode.hpp>
#endif
#include <gua/utils/string_utils.hpp>

// external headers
#include <fstream>
#include <sstream>
#include <iostream>
#include <map>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

DotGenerator::DotGenerator()
    : parse_data_(), added_nodes_(), node_count_(0) {}

////////////////////////////////////////////////////////////////////////////////

DotGenerator::~DotGenerator() {}

////////////////////////////////////////////////////////////////////////////////

void DotGenerator::parse_graph(SceneGraph const* graph) {
  // reset
  node_count_ = 0;
  added_nodes_.clear();

  parse_data_ += "graph guacamole { \n";

  graph->accept(*this);

  parse_data_ += "} \n";
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(Node* node) {
  pre_node_info(node);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#FFDD55\"";
  fillcolor += "]";

  post_node_info(node, fillcolor);

  for (auto child : node->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(TransformNode* cam) {
  pre_node_info(cam);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#888888\"";
  fillcolor += "]";

  post_node_info(cam, fillcolor);

  for (auto child : cam->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(GeometryNode* geometry) {
  pre_node_info(geometry);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#CCCCCC\"";
  if (geometry->get_filename() != "")
    parse_data_ += "| geometry: " + geometry->get_filename();
  if (geometry->get_material() != "")
    parse_data_ += "| material: " + geometry->get_material();

  fillcolor += "]";

  post_node_info(geometry, fillcolor);

  for (auto child : geometry->children_)
    child->accept(*this);
}


////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(VolumeNode* volume) {
	pre_node_info(volume);

	std::string fillcolor("[fillcolor =");
	fillcolor += " \"#CCEECC\"";
	if (volume->data.get_volume() != "")
		parse_data_ += "| volume: " + volume->data.get_volume();

	fillcolor += "]";

	post_node_info(volume, fillcolor);

	for (auto child : volume->children_)
		child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(PointLightNode* pointlight) {
  pre_node_info(pointlight);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#FFDD55\"";
  fillcolor += "]";

  post_node_info(pointlight, fillcolor);

  for (auto child : pointlight->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(ScreenNode* screen) {
  pre_node_info(screen);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#55DDFF\"";
  fillcolor += "]";

  post_node_info(screen, fillcolor);

  for (auto child : screen->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(SpotLightNode* spot) {
  pre_node_info(spot);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#FFDD55\"";
  fillcolor += "]";

  post_node_info(spot, fillcolor);

  for (auto child : spot->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(RayNode* ray) {
  pre_node_info(ray);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#DDFF22\"";
  fillcolor += "]";

  post_node_info(ray, fillcolor);

  for (auto child : ray->children_)
    child->accept(*this);
}

#if 0
////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(physics::RigidBodyNode* rb) {
  pre_node_info(rb);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#FF2222\"";
  parse_data_ += "| mass: " + string_utils::to_string<float>(rb->mass());
  parse_data_ +=
      "| friction: " + string_utils::to_string<float>(rb->friction());
  parse_data_ +=
      "| restitution: " + string_utils::to_string<float>(rb->restitution());
  fillcolor += "]";

  post_node_info(rb, fillcolor);

  for (auto child : rb->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(physics::CollisionShapeNode* shape) {
  pre_node_info(shape);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#AAFFAA\"";
  if (shape->data.get_shape() != "")
    parse_data_ += "| shape: " + shape->data.get_shape();
  fillcolor += "]";

  post_node_info(shape, fillcolor);

  for (auto child : shape->children_)
    child->accept(*this);
}
#endif

////////////////////////////////////////////////////////////////////////////////
/* virtual */ void DotGenerator::visit(TexturedQuadNode* node) {
  pre_node_info(node);

  std::string fillcolor("[fillcolor =");
  fillcolor += " \"#A52D9F\"";
  fillcolor += "]";

  post_node_info(node, fillcolor);

  for (auto child : node->children_)
    child->accept(*this);
}

////////////////////////////////////////////////////////////////////////////////

void DotGenerator::pre_node_info(Node* node) {
  int current_depth(node->get_depth());
  std::stringstream node_name;
  node_name << node_count_;

  if (added_nodes_.find(current_depth - 1) != added_nodes_.end()) {
    std::stringstream previous_name;
    previous_name << added_nodes_[current_depth - 1];
    parse_data_ += "    " + previous_name.str();
    parse_data_ += " -- " + node_name.str();
  } else {
    parse_data_ += "    " + node_name.str();
  }

  parse_data_ += ";\n    " + node_name.str() + " [label= \"{" +
                 (node->get_name() == "/" ? "root" : node->get_name());
}

////////////////////////////////////////////////////////////////////////////////

void DotGenerator::post_node_info(Node* node, std::string const& fillcolor) {
  parse_data_ += "}\"]" + std::string(" [shape = record]") + " [style=filled] ";

  parse_data_ += fillcolor + ";\n";

  assert(node_count_ < std::numeric_limits<int>::max());

  added_nodes_[node->get_depth()] = int(node_count_);
  ++node_count_;
}

////////////////////////////////////////////////////////////////////////////////

void DotGenerator::save(std::string const& path_to_file) const {

  std::fstream file;
  file.open(path_to_file, std::fstream::out);

  if (file.good()) {
    file.write(parse_data_.c_str(), parse_data_.size());
    file.close();
  } else {
    Logger::LOG_WARNING << "Failed to save dot graph: Failed to open file \"" << path_to_file << "\"." << std::endl;
  }
}

////////////////////////////////////////////////////////////////////////////////

}
