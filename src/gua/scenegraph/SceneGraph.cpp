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
#include <gua/scenegraph/SceneGraph.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/scenegraph/GroupNode.hpp>
#include <gua/utils/PathParser.hpp>
#include <gua/utils/DotGenerator.hpp>
#include <gua/utils/Mask.hpp>
#include <gua/utils/logger.hpp>

// external headers
#include <iostream>

namespace gua {

SceneGraph::SceneGraph(std::string const& name)
    : root_(new GroupNode("/", math::mat4::identity())),
      name_(name) {}

SceneGraph::SceneGraph(SceneGraph const& graph)
    : root_(graph.root_ ? graph.root_->deep_copy() : nullptr),
      name_(graph.name_) {}

void SceneGraph::remove_node(std::string const& path_to_node) {

    std::shared_ptr<Node> const& searched_node(find_node(path_to_node));

    if (searched_node && searched_node->get_parent()) {
        searched_node->get_parent()->remove_child(searched_node);
    }
}

void SceneGraph::remove_node(std::shared_ptr<Node> const& to_remove) {
    if (to_remove->get_parent()) {
        to_remove->get_parent()->remove_child(to_remove);
    }
}

void SceneGraph::set_name(std::string const& name) {
  name_ = name;
}

std::string const& SceneGraph::get_name() const {
  return name_;
}

void SceneGraph::set_root(std::shared_ptr<Node> const& root) {
  root_ = root;
}

std::shared_ptr<Node> const& SceneGraph::get_root() const {
  return root_;
}

std::shared_ptr<Node> SceneGraph::operator[](std::string const& path_to_node) const {

    auto result(find_node(path_to_node, "/"));

    if (result == nullptr)
        WARNING("Unable to get node: A node at location %s doesn't exist!",
                path_to_node.c_str());
    return result;
}

SceneGraph const& SceneGraph::operator=(SceneGraph const& rhs) {

    root_ = rhs.root_ ? rhs.root_->deep_copy() : nullptr;

    return *this;
}

void SceneGraph::to_dot_file(std::string const& file) const {
    DotGenerator generator;
    generator.parse_graph(this);
    generator.save(file);
}

void SceneGraph::update_cache() const {
    if (root_) {
        root_->update_cache();
    }
}

std::shared_ptr<Node> SceneGraph::find_node(std::string const& path_to_node,
                            std::string const& path_to_start) const {

    PathParser parser;
    parser.parse(path_to_node);

    std::shared_ptr<Node> to_be_found(path_to_start == "/"
                          ? root_
                          : find_node(path_to_start));

    for (auto const& node_name : parser.get_parsed_path()) {
      for (auto child : to_be_found->get_children()) {
            if (child->get_name() == node_name) {
                to_be_found = child;
                break;
            }
        }

        if (to_be_found->get_name() != node_name) {
            return nullptr;
        }
    }

    return to_be_found;
}

bool SceneGraph::has_child(std::shared_ptr<Node> const& parent,
                           std::string const & child_name) const {

    auto children(parent->get_children());

    for (auto child : children) {
        if (child->get_name() == child_name) {
            return true;
        }
    }
    return false;
}

void SceneGraph::accept(NodeVisitor & visitor) const { root_->accept(visitor); }

std::set<PickResult> const SceneGraph::ray_test(RayNode const& ray,
                                                PickResult::Options options,
                                                std::string const& mask) {
    return root_->ray_test(ray, options, mask);
}

}
