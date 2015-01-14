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
#include <gua/renderer/PipelineDescription.hpp>
#include <gua/renderer/enums.hpp>
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

  enum class ProjectionMode {
    PERSPECTIVE,
    ORTHOGRAPHIC
  };

  struct Configuration {

    // if set to false, this camera won't render anything
    GUA_ADD_PROPERTY(bool,            enabled,                true);

    // the camera renders a view into this scenegraph. The camera itself does
    // not neccessarily has to be in the very same scenegraph.
    GUA_ADD_PROPERTY(std::string,     scene_graph_name,       "unknown_scene_graph");

    // limits the rendered object to a set defined by the mask
    GUA_ADD_PROPERTY(Mask,            mask,                   Mask());

    // a user-defined view id, can be used to customize material parameters for
    // objects rendered by this camera
    GUA_ADD_PROPERTY(int,             view_id,                0);

    // whether this camera renders in perspective or orthographic mode
    GUA_ADD_PROPERTY(ProjectionMode,  mode,                   ProjectionMode::PERSPECTIVE);

    // viewing setup and stereo configuration
    GUA_ADD_PROPERTY(bool,            enable_stereo,          false);
    GUA_ADD_PROPERTY(float,           eye_dist,               0.07f);
    GUA_ADD_PROPERTY(float,           eye_offset,             0.f);
    GUA_ADD_PROPERTY(std::string,     left_screen_path,       "unknown_screen");
    GUA_ADD_PROPERTY(std::string,     right_screen_path,      "unknown_screen");
    GUA_ADD_PROPERTY(CameraMode,      mono_mode,              CameraMode::CENTER);

    // the rendering is performed with thid resolution. Usually it should match
    // the output window's size.
    GUA_ADD_PROPERTY(math::vec2ui,    resolution,             math::vec2ui(800, 600));

    // the final image of this camera will be stored in the texture database
    // with this name. if enable_stereo is set to true, two images with postfixes
    // _left and _right will be stored
    GUA_ADD_PROPERTY(std::string,     output_texture_name,    "");

    // if set to a non-empty string the produced texture will be displayed in a
    // window of the window database with this name
    GUA_ADD_PROPERTY(std::string,     output_window_name,     "");

    // clipping
    GUA_ADD_PROPERTY(float,           near_clip,              0.1f);
    GUA_ADD_PROPERTY(float,           far_clip,               1000.0f);

    // culling
    GUA_ADD_PROPERTY(bool,            enable_frustum_culling, true);

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
  CameraNode()
    : application_fps_(0.f)
    , rendering_fps_(0.f) {}

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
             std::shared_ptr<PipelineDescription> const& description = PipelineDescription::make_default(),
             Configuration const& configuration = Configuration(),
             math::mat4 const& transform = math::mat4::identity());

  std::vector<std::shared_ptr<CameraNode>> const& get_pre_render_cameras() const {
    return pre_render_cameras_;
  }

  std::vector<std::shared_ptr<CameraNode>>& get_pre_render_cameras() {
    return pre_render_cameras_;
  }

  void set_pre_render_cameras(std::vector<std::shared_ptr<CameraNode>> const& cams) {
    pre_render_cameras_ = cams;
  }


  std::shared_ptr<PipelineDescription> const& get_pipeline_description() const {
    return pipeline_description_;
  }

  std::shared_ptr<PipelineDescription>& get_pipeline_description() {
    return pipeline_description_;
  }

  void set_pipeline_description(std::shared_ptr<PipelineDescription> const& pipeline_description) {
    pipeline_description_ = pipeline_description;
  }


  float get_application_fps() const {
    return application_fps_;
  }

  float get_rendering_fps() const {
    return rendering_fps_;
  }

  void set_application_fps(float application_fps) {
    application_fps_ = application_fps;
  }

  void set_rendering_fps(float rendering_fps) {
    rendering_fps_ = rendering_fps;
  }

  SerializedCameraNode serialize() const;

  /**
   * Accepts a visitor and calls concrete visit method.
   *
   * This method implements the visitor pattern for Nodes.
   *
   * \param visitor  A visitor to process the CameraNode's data.
   */
  void accept(NodeVisitor& visitor) override;

 private:

  float application_fps_;
  float rendering_fps_;

  std::shared_ptr<Node> copy() const override;

  // based on this description the rendering is performed
  std::shared_ptr<PipelineDescription> pipeline_description_;

  // access this members only from the rendering thread!
  std::shared_ptr<Pipeline> rendering_pipeline_;

  std::vector<std::shared_ptr<CameraNode>> pre_render_cameras_;
};

struct GUA_DLL SerializedCameraNode {
  CameraNode::Configuration             config;
  math::mat4                            transform;
  std::shared_ptr<Pipeline>             rendering_pipeline;
  std::shared_ptr<PipelineDescription>  pipeline_description;

  std::vector<SerializedCameraNode>     pre_render_cameras;
};


} // namespace node {
} // namespace gua {

#endif  // GUA_CAMERA_NODE_HPP
