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

#ifndef GUA_CAMERA_NODE_HPP
#define GUA_CAMERA_NODE_HPP

#include <gua/platform.hpp>
#include <gua/node/Node.hpp>
#include <gua/renderer/Pipeline.hpp>
#include <gua/utils/Mask.hpp>
#include <gua/utils/configuration_macro.hpp>

namespace gua {
namespace node {

struct SerializedCameraNode;

/**
 * This class is used to represent a camera in the SceneGraph.
 *
 * \ingroup gua_scenegraph
 */
class GUA_DLL CameraNode : public Node {
 public:

  enum ProjectionMode {
    PERSPECTIVE,
    ORTHOGRAPHIC
  };

  struct Configuration {
    GUA_ADD_PROPERTY(PipelineDescription, pipeline_description, Pipeline::make_default());

    GUA_ADD_PROPERTY(float,          eye_dist,          0.07f);
    GUA_ADD_PROPERTY(float,          eye_offset,        0.f);
    GUA_ADD_PROPERTY(std::string,    left_screen_path,  "unknown_screen");
    GUA_ADD_PROPERTY(std::string,    right_screen_path, "unknown_screen");
    GUA_ADD_PROPERTY(std::string,    scene_graph_name,  "unknown_scene_graph");
    GUA_ADD_PROPERTY(ProjectionMode, mode,              PERSPECTIVE);
    GUA_ADD_PROPERTY(Mask,           mask,              Mask());

    // if set to false, this camera won't render anything
    GUA_ADD_PROPERTY(bool, enabled, true);

    GUA_ADD_PROPERTY(bool, enable_stereo, false);

    // the final image of this camera will be stored in the texture database
    // with this name. if enable_stereo is set to true, two images with postfixes
    // _left and _right will be stored
    GUA_ADD_PROPERTY(std::string, output_texture_name, "gua_pipeline");
    GUA_ADD_PROPERTY(std::string, output_window_name,  "gua_window");

    // stereo configuration
    GUA_ADD_PROPERTY(math::vec2ui, resolution, math::vec2ui(800, 600));

    // various display options
    GUA_ADD_PROPERTY(bool, enable_ray_display, false);
    GUA_ADD_PROPERTY(bool, enable_bbox_display, false);

    // clipping
    GUA_ADD_PROPERTY(float, near_clip, 0.1f);
    GUA_ADD_PROPERTY(float, far_clip, 1000.0f);

    // culling
    GUA_ADD_PROPERTY(bool, enable_frustum_culling, true);

    // convenience access to screen
    void set_screen_path(std::string const& path) {
      left_screen_path() = right_screen_path() = path;
    }

    std::string const& get_screen_path() {
      return get_left_screen_path();
    }
  };

  /**
   * The CameraNode's configuration.
   */
  Configuration config;

  /**
   * Constructor.
   *
   * This constructs an empty CameraNode.
   *
   */
  CameraNode() {}

  /**
   * Constructor.
   *
   * This constructs a CameraNode with the given parameters.
   *
   * \param name           The name of the new CameraNode.
   * \param configuration  A configuration struct to define the CameraNode's
   *                       properties.
   * \param transform      A matrix to describe the CameraNode's
   *                       transformation. By default, the CameraNode is aligned
   *                       with the xy-plane and facing in +z direction.
   */
  CameraNode(std::string const& name,
             Configuration const& configuration = Configuration(),
             math::mat4 const& transform = math::mat4::identity());

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the CameraNode's data.
   */
  void accept(NodeVisitor& visitor) override;

  std::shared_ptr<SerializedCameraNode> serialize() const;

 private:

  std::shared_ptr<Node> copy() const override;

};

  
struct GUA_DLL SerializedCameraNode {
  CameraNode::Configuration config;
  math::mat4                transform;
};

} // namespace node {
} // namespace gua {

#endif  // GUA_CAMERA_NODE_HPP
