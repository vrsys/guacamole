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
#include <gua/node/CameraNode.hpp>

// guacamole header
#include <gua/scenegraph/NodeVisitor.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {
namespace node {

////////////////////////////////////////////////////////////////////////////////

CameraNode::CameraNode(std::string const& name,
                       std::shared_ptr<PipelineDescription> const& description,
                       Configuration const& configuration,
                       math::mat4 const& transform)
  : Node(name, transform), 
    config(configuration),
    pipeline_description_(description),
    application_fps_(0.f),
    rendering_fps_(0.f) 
{}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void CameraNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> CameraNode::copy() const {
    return std::make_shared<CameraNode>(get_name(), pipeline_description_, config, get_transform());
}

////////////////////////////////////////////////////////////////////////////////

SerializedCameraNode CameraNode::serialize() const 
{
  SerializedCameraNode s = { config, get_world_transform(), uuid() };

  for (auto const& cam: pre_render_cameras_) {
      s.pre_render_cameras.push_back(cam->serialize());
  }

  s.pipeline_description = pipeline_description_;

  return s;
}

////////////////////////////////////////////////////////////////////////////////

}
}
