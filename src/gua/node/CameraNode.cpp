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
#include <gua/scenegraph.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Logger.hpp>

namespace gua {
namespace node {

////////////////////////////////////////////////////////////////////////////////

CameraNode::CameraNode(std::string const& name,
                       std::shared_ptr<PipelineDescription> const& description,
                       Configuration const& configuration,
                       math::mat4 const& transform)
    : Node(name, transform), config(configuration)
    , rendering_pipeline_(std::make_shared<Pipeline>())
    , pipeline_description_(description)
    , application_fps_(0.f)
    , rendering_fps_(0.f) {}

/* virtual */ void CameraNode::accept(NodeVisitor& visitor) {

  visitor.visit(this);
}

////////////////////////////////////////////////////////////////////////////////

Frustum CameraNode::get_frustum(SceneGraph const& graph, CameraMode mode) const {
    return make_frustum(graph, get_world_transform(), config, mode);
}

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> CameraNode::copy() const {
    return std::make_shared<CameraNode>(get_name(), pipeline_description_, config, get_transform());
}

////////////////////////////////////////////////////////////////////////////////

SerializedCameraNode CameraNode::serialize() const {
    SerializedCameraNode s = {config, get_world_transform(), rendering_pipeline_};

    for (auto const& cam: pre_render_cameras_) {
        s.pre_render_cameras.push_back(cam->serialize());
    }

    s.pipeline_description = pipeline_description_;

    return s;
}

////////////////////////////////////////////////////////////////////////////////

Frustum CameraNode::make_frustum(SceneGraph const& graph, math::mat4 const& camera_transform, CameraNode::Configuration const& config, CameraMode mode) {
    std::string screen_name(mode != CameraMode::RIGHT ? config.left_screen_path() : config.right_screen_path());
    auto screen_it(graph[screen_name]);
    auto screen(std::dynamic_pointer_cast<node::ScreenNode>(screen_it));
    if (!screen) {
        Logger::LOG_WARNING << "Cannot make Frustum: No valid screen specified" << std::endl;
        return Frustum();
    }

    auto transform(camera_transform);

    if (config.get_enable_stereo()) {
        if (mode != CameraMode::RIGHT) {
            transform *= scm::math::make_translation(math::float_t(config.eye_offset() - 0.5f * config.eye_dist()), math::float_t(0), math::float_t(0));
        } else {
            transform *= scm::math::make_translation(math::float_t(config.eye_offset() + 0.5f * config.eye_dist()), math::float_t(0), math::float_t(0));
        }
    }

    if (config.mode() == node::CameraNode::ProjectionMode::PERSPECTIVE) {
        return Frustum::perspective(
            transform, screen->get_scaled_world_transform(), 
            config.near_clip(), config.far_clip()
        );
    } 

    return Frustum::orthographic(
        transform, screen->get_scaled_world_transform(), 
        config.near_clip(), config.far_clip()
    );
}

////////////////////////////////////////////////////////////////////////////////

}
}
