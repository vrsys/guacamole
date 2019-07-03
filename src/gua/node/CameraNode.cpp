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
#include <gua/databases/TextureDatabase.hpp>

namespace gua
{
namespace node
{
////////////////////////////////////////////////////////////////////////////////

CameraNode::CameraNode(std::string const& name, std::shared_ptr<PipelineDescription> const& description, Configuration const& configuration, math::mat4 const& transform)
    : Node(name, transform), config(configuration), pipeline_description_(description)
{
}

////////////////////////////////////////////////////////////////////////////////

/* virtual */ void CameraNode::accept(NodeVisitor& visitor) { visitor.visit(this); }

////////////////////////////////////////////////////////////////////////////////

Frustum CameraNode::get_rendering_frustum(SceneGraph const& graph, CameraMode mode) const { return make_frustum(graph, get_world_transform(), config, mode, false); }

////////////////////////////////////////////////////////////////////////////////

Frustum CameraNode::get_culling_frustum(SceneGraph const& graph, CameraMode mode) const { return make_frustum(graph, get_world_transform(), config, mode, true); }

////////////////////////////////////////////////////////////////////////////////

std::shared_ptr<Node> CameraNode::copy() const { return std::make_shared<CameraNode>(*this); }

////////////////////////////////////////////////////////////////////////////////

std::map<size_t, CameraNode*> SerializedCameraNode::camera_nodes = std::map<size_t, CameraNode*>();

SerializedCameraNode CameraNode::serialize() const
{
    SerializedCameraNode s = {config, get_world_transform(), uuid()};

    for(auto const& cam : pre_render_cameras_)
    {
        s.pre_render_cameras.push_back(cam->serialize());
    }

    s.pipeline_description = pipeline_description_;

	s.parents_transform = get_parent()->get_world_transform();
    s.camera_node_name = get_name();
  
	if (SerializedCameraNode::camera_nodes[s.uuid] == nullptr) {
	    SerializedCameraNode::camera_nodes[s.uuid] = (CameraNode*) this;
	}

    return s;
}

////////////////////////////////////////////////////////////////////////////////

Frustum CameraNode::make_frustum(SceneGraph const& graph, math::mat4 const& camera_transform, CameraNode::Configuration const& config, CameraMode mode, bool use_alternative_culling_screen)
{
    std::string screen_name(config.get_alternative_frustum_culling_screen_path());

    if(screen_name == "" || !use_alternative_culling_screen)
    {
        screen_name = mode != CameraMode::RIGHT ? config.left_screen_path() : config.right_screen_path();
    }

    auto screen_it(graph[screen_name]);
    auto screen(std::dynamic_pointer_cast<node::ScreenNode>(screen_it));
    if(!screen)
    {
        Logger::LOG_WARNING << "Cannot make Frustum: No valid screen specified" << std::endl;
        return Frustum();
    }

    auto eye_transform(camera_transform);
    auto screen_transform(screen->get_scaled_world_transform());

    float camera_scale(scm::math::length(math::vec3(camera_transform[8], camera_transform[9], camera_transform[10])));
    float clipping_offset(0.f);

    if(config.get_enable_stereo())
    {
        // assure same clipping for left and right eye
        math::vec4 eye_separation(camera_transform * math::vec4(config.eye_dist(), 0.f, 0.f, 0.f));
        math::vec4 screen_direction(screen_transform * math::vec4(0.f, 0.f, -1.f, 0.f));

        math::vec3 eye_separation_in_screen_direction(scm::math::dot(eye_separation, screen_direction) / scm::math::length_sqr(screen_direction) * screen_direction);

        float eye_dist_in_screen_direction(scm::math::length(eye_separation_in_screen_direction) / camera_scale);

        // left eye is closer to screen than left eye
        if(eye_separation.x * screen_direction.x + eye_separation.y * screen_direction.y + eye_separation.z * screen_direction.z > 0)
        {
            // move left eye clipping towards screen
            if(mode != CameraMode::RIGHT)
                clipping_offset = eye_dist_in_screen_direction;
        }
        else
        {
            // move right eye clipping towards screen
            if(mode == CameraMode::RIGHT)
                clipping_offset = eye_dist_in_screen_direction;
        }

        if(mode != CameraMode::RIGHT)
        {
            eye_transform *= scm::math::make_translation(math::float_t(config.eye_offset() - 0.5f * config.eye_dist()), math::float_t(0), math::float_t(0));
        }
        else
        {
            eye_transform *= scm::math::make_translation(math::float_t(config.eye_offset() + 0.5f * config.eye_dist()), math::float_t(0), math::float_t(0));
        }
    }

    if(config.mode() == node::CameraNode::ProjectionMode::PERSPECTIVE)
    {
        return Frustum::perspective(eye_transform, screen_transform, config.near_clip() / camera_scale + clipping_offset, config.far_clip() / camera_scale + clipping_offset);
    }

    return Frustum::orthographic(eye_transform, screen_transform, config.near_clip() / camera_scale + clipping_offset, config.far_clip() / camera_scale + clipping_offset);
}

////////////////////////////////////////////////////////////////////////////////

void CameraNode::set_scenegraph(SceneGraph* scenegraph)
{
    if(scenegraph_)
    {
        scenegraph_->remove_camera_node(this);
    }

    Node::set_scenegraph(scenegraph);

    if(scenegraph_)
    {
        scenegraph_->add_camera_node(this);
    }
}

void CameraNode::set_pre_render_cameras(std::vector<std::shared_ptr<CameraNode>> const& cams)
{
    for(auto const& cam : cams)
    {
        auto tex_name = cam->config.get_output_texture_name();
        if(tex_name != "")
        {
            // insert dummy textures
            math::vec2ui size(std::max(1U, cam->config.resolution().x), std::max(1U, cam->config.resolution().y));
            scm::gl::sampler_state_desc state(scm::gl::FILTER_MIN_MAG_LINEAR, scm::gl::WRAP_MIRRORED_REPEAT, scm::gl::WRAP_MIRRORED_REPEAT);
            auto color = std::make_shared<Texture2D>(size.x, size.y, scm::gl::FORMAT_RGB_32F, 1, state);
            TextureDatabase::instance()->add_if_not_element(tex_name, color);

            auto depth = std::make_shared<Texture2D>(size.x, size.y, scm::gl::FORMAT_D24_S8, 1, state);
            TextureDatabase::instance()->add_if_not_element(tex_name + "_depth", depth);
        }
    }
    pre_render_cameras_ = cams;
}

////////////////////////////////////////////////////////////////////////////////

} // namespace node
} // namespace gua
