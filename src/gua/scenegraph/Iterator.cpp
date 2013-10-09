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
#include <gua/scenegraph/Iterator.hpp>

// guacamole headers
#include <gua/scenegraph/SceneGraph.hpp>
#include <gua/scenegraph/Node.hpp>
#include <gua/utils/Profiler.hpp>
#include <gua/utils/logger.hpp>

namespace gua {

const std::string SceneGraph::Iterator::end_name_("end");
const math::mat4 SceneGraph::Iterator::end_transform_(math::mat4::identity());

SceneGraph::Iterator::Iterator()
    : current_node_(nullptr),
      start_node_(nullptr),
      current_depth_(0) {}

SceneGraph::Iterator::Iterator(std::shared_ptr<Node> const& node)
    : current_node_(node),
      start_node_(node),
      current_depth_(0) {}

void SceneGraph::Iterator::operator++() {
  next();
}

bool SceneGraph::Iterator::operator==(Iterator const& rhs) {
  return (current_node_ == rhs.current_node_);
}

bool SceneGraph::Iterator::operator!=(Iterator const& rhs) {
  return (current_node_ != rhs.current_node_);
}

SceneGraph::Iterator::operator bool() const { return current_node_ != nullptr; }

std::shared_ptr<Node> const& SceneGraph::Iterator::operator*() const { return current_node_; }

std::shared_ptr<Node> const& SceneGraph::Iterator::operator->() const { return current_node_; }

void SceneGraph::Iterator::next() {
  if (!current_node_->get_children().empty()) {
    current_node_ = current_node_->get_children().front();
    ++current_depth_;
  } else {
    bool found_next(false);
    while (!found_next) {
      if (current_node_ != start_node_) {
        auto neighbour(get_neighbour(current_node_));
        if (neighbour) {
          current_node_ = neighbour;
          found_next = true;
        } else {
          current_node_ = current_node_->get_parent_shared();
          --current_depth_;
        }
      } else {
        *this = Iterator();
        break;
      }
    }
  }
}

std::shared_ptr<Node> SceneGraph::Iterator::get_neighbour(std::shared_ptr<Node> const& to_be_checked) {
  auto end(to_be_checked->get_parent()->get_children().end());
  for (auto child(to_be_checked->get_parent()->get_children().begin());
       child != end;
       ++child) {

    if (*child == to_be_checked) {
      if (++child != end) {
        return *child;
      } else
        break;
    }
  }

  return nullptr;
}

}
